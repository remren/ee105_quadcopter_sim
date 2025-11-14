# EE105: Quadcopter Sim
## System Diagram
- DMR: Desired Motion Rate
- RRE: Rotation Rate Error
- MPC: Motor Power Command
- MTT: Motor Thrust and Torque
- QRR: Quadcopter Rotation Rate
- MSR: Measured Sensor Rate

<pre>
--(DMR)->[+    ]--(RRE)->[PID Controller]--(MPC)->[Motor Dynamics]--(MTT)->[Quadcopter Dynamics]-|
         [  -  ]                                                                                 |
            ^                                                                                    |
            |                                                                                    |
            ------(MRR)------------[Sensor Dynamics]<---------------(QRR)------------------------|
</pre>

## TODO:
- [] Primary Control Loop
    - [] Motors
    - [] Sensors
- [] Go Beyond
    - [] Battery
    - [] Godot
    
### Notes
- Characterizing Motor
    - Time response of motor was obtained by taking spectrogram of test run of motor. Motor taken from 280 Hz at 11.5% motor power (?) to 400 Hz at 50% power.
    - Assume MPC, motor power command, issued was "instant".
        - Essentially characterize a unit step function in Laplace domain, so x(t) = u(t)
    - Data from manual shows it took 0.09 seconds to get to 400 Hz, from a "step" function MPC.
        - Output then is: y(t) = (1 - e^(-t/tau)) * x(t) (gonna make convolution \conv, not *, * is standard mult.)
        - So, output is just: y(t) = u(t) - u(t)*e^(-t/tau)
            - *** Tangent *** OK. this is where i don't get why the manual chose these values. pg. 196, "\tau is defined as the time ... motor output reaches 95% of desired value divided by 3." Why 3? Trying to think of reasons, eg. RC circuits/filters, but those usually use 2\pi, not 3? But those are using different components, so maybe 3 is a good approximation.
            - eg. in an LC circuit, tau is
                - $f_c = \frac{1}{2\pi}\sqrt{\tau}$ || f_c = 1 / (2pi * sqrt(tau))
                - $\tau = \frac{1}{(f_c * 2\pi)^2}$ || tau = 1 / (2pi * f_c)^2
                - 1 / (2pi)^2 = 0.0253? wat.
            - Motor Time Constant Resource (63%): https://www.nidec.com/en/technology/motor/glossary/item/time_constant/
        - Resulting Y(s) = 1/s - 1/(s + 1/tau)
                         = (s + 1/tau) / (s * (s + 1/tau)) - s / (s * (s + 1/tau))
                         = 1 / (... ok what still pg.196, use unit step -> characterize as 1/s using extra s..
                         = 1 / (tau * s + 1)
