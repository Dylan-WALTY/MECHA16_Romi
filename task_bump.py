from pyb import ExtInt, Pin, enable_irq, disable_irq 
from array import array 

class task_crash: 

    def __init__(self, crash_detect, pins): 

        # A Queue used to store crash detection events 
        self._cd = crash_detect    

        self._pins = {pin.pin(): pin for pin in pins}  # Map pin numbers to Pin objects
        
        # A dictionary used to map pin numbers (ISR Lines) to ExtInt objects 
        self._callbacks = {pin.pin(): ExtInt( 
            pin, ExtInt.IRQ_FALLING, Pin.PULL_UP, self.callback) for pin in pins} 

        # An array of two 16-bit integers used to store current and previous DB 
        # states 
        #                               ___________ Current DB state 
        #                              /\      ____ Previous DB state 
        #                             /  \    /  \ 
        self._db_mask = array("H", [0x0000, 0x0000]) 

    def callback(self,ISR_src): 

        # Set the debounce state to include the channel which triggered this 
        # ISR cycle 
        self._db_mask[0] |= 1<<ISR_src 

        # Disable the callback on this channel so that no more interupts 
        # occur until after debounce period 
        self._callbacks[ISR_src].disable() 

        # Put into the crash detect flag so that other tasks can know 
        # a bumper was hit 
        if self._pins[ISR_src].value() == 0:
            self._cd.put(ISR_src, in_ISR=True) 

    def run(self): 

        while True: 

            # Check which channels have pending debounce by examining 
            # the mask representing debounce states. Reenable any channels 
            # that are due 
            for ISR_src in range(16): 
                if self._db_mask[1] & (1 << ISR_src): 
                    self._callbacks[ISR_src].enable() 

            # Begin critical section 
            irq_state = disable_irq() 

            # Shift the current debounce state to previous state and 
            # reset the current state to zero 
            self._db_mask[1], self._db_mask[0] = self._db_mask[0], 0x0000 

            # End critical section 
            enable_irq(irq_state) 

            yield 