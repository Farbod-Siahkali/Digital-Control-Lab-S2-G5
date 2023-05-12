<div class="markdown prose w-full break-words dark:prose-invert light">
    <h1>Digital Control Lab</h1>
    <p>This repository contains the source code and documentation for a lab project on digital control. The project
        involves an STM32 microcontroller and a DC motor.</p>
    <h2>Sessions</h2>
    <h3>Session 1</h3>
    <p>In Session 1, we wrote a simple program to toggle the state of an LED on the STM32 board. Additionally, we set up
        an external interrupt on pin PB3 to change the blinking rate of the LED.</p>
    <h3>Session 2 - Phase 1</h3>
    <p>During this session, we continued working with external interrupts and timers. We configured an interrupt on pin
        PB3 to start and stop a timer. We also set up another timer to generate an interrupt at a fixed frequency.</p>
    <h3>Session 2 - Phase 2</h3>
    <p>In the second phase of Session 2, we made modifications to the code to use a different GPIO pin and timer for
        generating the interrupt.</p>
    <h3>Session 3</h3>
    <p>Session 3 focused on controlling the speed and direction of a DC motor using pulse-width modulation (PWM) and
        quadrature encoders. We implemented techniques to measure the speed and position of the motor.</p>
    <h3>Session 4</h3>
    <p>In Session 4, we utilized the step response of the motor to determine the motor's parameters (k and tau) using a
        method called "system identification."</p>
    <h3>Session 5 - Control of DC Motor</h3>
    <p>In the fifth session, we worked on controlling a DC motor using digital techniques. We used an STM32
        microcontroller and implemented a digital control algorithm to regulate the motor's speed. The algorithm
        involved proportional-integral (PI) control with a setpoint of 700 RPM. We calculated the control output based
        on the error between the setpoint and the actual motor speed, and adjusted the motor's direction and PWM signal
        accordingly. The control algorithm was executed at a sampling time of 7.62 ms. The code for this session can be
        found in the <code>main.c</code> file.</p>
    <h4>Wiring</h4>
    <p>The DC motor was connected to the STM32 microcontroller as follows:</p>
    <ul>
        <li>Connect the motor's positive terminal to the STM32's PWM output pin (TIM11 Channel 1).</li>
        <li>Connect the motor's negative terminal to the GND pin of the STM32.</li>
        <li>Connect the direction control pin (pin PB4) of the STM32 to the motor's direction control pin.</li>
        <li>Connect the A signal pin (pin PE6) of the STM32 to the motor's quadrature encoder A signal pin.</li>
        <li>Connect the encoder's common ground to the GND pin of the STM32.</li>
    </ul>
    <h4>Oscilloscope Picture</h4>
    <p><img src="oscilloscope_picture.png" alt="Oscilloscope Picture"></p>
    <p>Please refer to the picture above to see the oscilloscope waveform captured during the DC motor control process.
    </p>
    <h2>Contributors</h2>
    <ul>
        <li>Saba Samadi</li>
        <li>Navid Dehban</li>
    </ul>
</div>