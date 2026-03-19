from pyb import ExtInt, Pin, enable_irq, disable_irq
from array import array


class task_crash:
    """Interrupt-driven bump sensor with two-stage software debounce.

    Three bump switches are each connected to a falling-edge external
    interrupt. When a switch fires the ISR records the pin number in a
    bitmask and immediately disables that interrupt channel. On the next
    task tick the previous cycle's bitmask is transferred to a secondary
    mask whose channels are then re-enabled, completing the debounce
    window. A contact is only registered if the pin remains low when the
    ISR fires, preventing false triggers from noise.

    Detected contacts are written to the crash_detect Queue for
    consumption by task_master.
    """

    def __init__(self, crash_detect, pins):
        """Configure ExtInt falling-edge interrupts for each bump pin."""
        self._cd   = crash_detect
        self._pins = {pin.pin(): pin for pin in pins}

        self._callbacks = {
            pin.pin(): ExtInt(pin, ExtInt.IRQ_FALLING, Pin.PULL_UP, self.callback)
            for pin in pins
        }

        # _db_mask[0]: pins that fired this cycle; _db_mask[1]: pending re-enable
        self._db_mask = array("H", [0x0000, 0x0000])

    def callback(self, ISR_src):
        """ISR: record hit, disable this channel, and enqueue if pin is still low."""
        self._db_mask[0] |= 1 << ISR_src
        self._callbacks[ISR_src].disable()
        if self._pins[ISR_src].value() == 0:
            self._cd.put(ISR_src, in_ISR=True)

    def run(self):
        while True:
            # Re-enable any channels that were disabled in the previous tick
            for ISR_src in range(16):
                if self._db_mask[1] & (1 << ISR_src):
                    self._callbacks[ISR_src].enable()

            # Atomically rotate the pending mask into the re-enable mask
            irq_state = disable_irq()
            self._db_mask[1], self._db_mask[0] = self._db_mask[0], 0x0000
            enable_irq(irq_state)

            yield
