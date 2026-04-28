

The Stall Detection algorithm works only when the device is programmed to operate in the smart tune Ripple Control decay mode. The EN\_STL bit has to be '1' to enable stall detection. Additionally, if any fault condition exists (UVLO, OCP, OL, OTSD etc.), stall detection will be disabled.

The algorithm compares the back-EMF between the rising and falling current quadrants by monitoring PWM off time and generates a value represented by the 12-bit register TRQ\_COUNT. The comparison is done in such a way that the TRQ\_COUNT value is practically independent of motor current, ambient temperature and supply voltage. Full step mode of operation is supported by this algorithm.

For a lightly loaded motor, the TRQ\_COUNT will be a non-zero value. As the motor approaches stall condition, TRQ\_COUNT will approach zero and can be used to detect stall condition. If at anytime TRQ\_COUNT falls below the stall threshold (represented by the 12-bit STALL\_TH register), the device will detect a stall and the STALL, STL and FAULT bits are latched high in the SPI register. To indicate stall detection fault on the nFAULT pin, the STL\_REP bit must be '1'. When the STL\_REP bit is '1', the nFAULT pin will be driven low when a stall is detected.

In the stalled condition, the motor shaft does not spin. The motor starts to spin again when the stall condition is removed and the motor ramps to its target speed. The nFAULT is released and the fault registers are cleared when a clear faults command is issued either via the CLR\_FLT bit or an nSLEEP reset pulse.

TRQ\_COUNT is calculated as an average torque count of the most recent four electrical half-cycles of a spinning motor. The calculated value is updated in the device CTRL8 and CTRL9 registers within the next 100 ns. The registers are unchanged until the next update. Subsequent updates happen every electrical half-cycle.

High motor coil resistance can result in low TRQ\_COUNT. The TRQ\_SCALE bit allows scaling up low TRQ\_COUNT values, for ease of further processing. If the initially calculated TRQ\_COUNT value is less than 500 and the TRQ\_SCALE bit is '1', then the TRQ\_COUNT is multiplied by a factor of 8. If the TRQ\_SCALE bit is '0', TRQ\_COUNT retains the value originally calculated by the algorithm.

Stall threshold can be set in two ways – either the user can write the STALL\_TH bits, or let the algorithm learn the stall threshold value using the stall learning process. The stall learning process is started by setting the STL\_LRN bit to '1'. The motor is intentionally stalled briefly to allow the algorithm to learn the ideal stall threshold. At the end of a successful learning, the STALL\_TH register is updated with the learnt stall threshold value. The STL\_LRN\_OK bit goes high after a successful learning.

A stall threshold learnt at one speed may not work well for another speed. It is recommended to re-learn the stall threshold every time the motor speed is changed considerably.