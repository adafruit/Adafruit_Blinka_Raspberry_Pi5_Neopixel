#include <iostream>
#include <map>
#include <time.h>
#include <pybind11/pybind11.h>

#include "piolib.h"
#include "ws2812.pio.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

// Each GPIO is driven by its own state machine, claimed on first use. A single
// shared SM would need pio_sm_config_xfer() + ws2812_program_init() on every
// switch between pins/buffer sizes; SM_CONFIG_XFER tears down and re-requests
// the DMA channel in the kernel on each call, which both corrupts concurrent
// multi-strip output (#3) and trips a "scheduling while atomic" kernel bug
// (raspberrypi/linux#7512). Per-pin SMs configure DMA once and then only
// stream via SM_XFER_DATA.
struct SmState {
    int sm;
    size_t last_size;
    struct timespec deadline;
};

static PIO pio{};
static int offset{-1};
static std::map<int, SmState> sm_states;

constexpr auto NS_PER_SECOND = 1'000'000'000l;
constexpr auto NS_PER_MS = 1'000'000l;

static void timespec_add_ns(struct timespec &out, const struct timespec &in, long ns) {
    out = in;
    out.tv_nsec += ns;
    if(out.tv_nsec > NS_PER_SECOND) {
        out.tv_nsec -= NS_PER_SECOND;
        out.tv_sec += 1;
    }
}

static void neopixel_write(py::object gpio_obj, py::buffer buf) {
    int gpio = py::getattr(gpio_obj, "_pin", gpio_obj).attr("id").cast<int>();
    py::buffer_info info = buf.request();

    if (!pio) {
        // (is safe to call twice)
        if (pio_init()) {
            throw std::runtime_error("pio_init() failed");
        }

        // can't use `pio0` macro as it will call exit() on failure!
        pio = pio_open(0);
        if (PIO_IS_ERR(pio)) {
            throw std::runtime_error(
                py::str("Failed to open PIO device (error {})")
                    .attr("format")(PIO_ERR_VAL(pio))
                    .cast<std::string>());
        }

        offset = pio_add_program(pio, &ws2812_program);
    }

    auto it = sm_states.find(gpio);
    if (it == sm_states.end()) {
        // `required=false`: with one SM per GPIO, exhaustion (a 5th pin) is
        // reachable, and pio_panic() would kill the whole process.
        int sm = pio_claim_unused_sm(pio, false);
        if (sm < 0) {
            throw std::runtime_error("No PIO state machine available");
        }
        pio_sm_clear_fifos(pio, sm);
        ws2812_program_init(pio, sm, offset, gpio, 800000.0, true);
        it = sm_states.emplace(gpio, SmState{sm, 0, {}}).first;
    } else {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &it->second.deadline, NULL);
    }
    SmState &st = it->second;

    size_t size = info.size * info.itemsize;

    // rp1pio can only DMA in 32-bit blocks.
    // Copy the data into a temporary vector, with redundant zeros at the end
    // then byteswap it so that the data comes out in the right order.
    uint8_t *data = reinterpret_cast<uint8_t *>(info.ptr);
    std::vector<uint32_t> vec;
    vec.resize((size + 3) / 4);
    size_t data_size = vec.size() * 4;
    memcpy(&vec[0], data, size);
    for (auto &i : vec)
        i = __builtin_bswap32(i);

    if (data_size > UINT16_MAX) {
        throw py::value_error("Too much data");
    }

    if (data_size != st.last_size) {
        if (pio_sm_config_xfer(pio, st.sm, PIO_DIR_TO_SM, data_size, 1)) {
            throw std::runtime_error("pio_sm_config_xfer() failed");
        }
        st.last_size = data_size;
    }
    if (pio_sm_xfer_data(pio, st.sm, PIO_DIR_TO_SM, data_size, &vec[0])) {
        throw std::runtime_error("pio_sm_xfer_data() failed");
    }

    // Track the earliest time at which we can start a fresh neopixel transmission.
    // This needs to be long enough that all bits have been clocked out (the FIFO can
    // contain 16 entries of 32 bits each, taking 640us to transmit) plus the ws2812
    // required idle time ("RET code") of 50usmin. These sum to around 700us, so the 1ms
    // delay is generous. The deadline is per-SM: chains on other pins transmit
    // independently and need no cross-pin gap.
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    timespec_add_ns(st.deadline, ts, NS_PER_MS);
}

static void free_pio(void) {
    if (!pio) {
        return;
    }
    if (offset <= 0) {
        pio_remove_program(pio, &ws2812_program, offset);
    };
    offset = -1;
    for (auto &entry : sm_states) {
        pio_sm_unclaim(pio, entry.second.sm);
    }
    sm_states.clear();
    pio_close(pio);
    pio = nullptr;
}

PYBIND11_MODULE(adafruit_raspberry_pi5_neopixel_write, m) {
    m.doc() = R"pbdoc(
        neopixel_write for pi5
        ----------------------

        .. currentmodule:: adafruit_raspberry_pi5_neopixel_write

        .. autosummary::
           :toctree: _generate

           neopixel_write
           free_pio
    )pbdoc";

    m.def("neopixel_write", &neopixel_write, py::arg("gpio"), py::arg("buf"),
          R"pbdoc(NeoPixel writing function)pbdoc");

    m.def("free_pio", &free_pio, R"pbdoc(Release any held PIO resource)pbdoc");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
