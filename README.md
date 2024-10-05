# Open Bike Power Meter

This project aims to provide an open plattform for crank-based power meters for bicycles.
It is designed to achieve accurate and stable power measurement with real-time data transmission via Bluetooth Low Energy - Cycling Power Service (BLE - CPS).

The implemented features so far include:
- Accelerometer-based auto-start
- Auto zero-offset compensation at startup
- Power saving in idle mode
- Calibration and runtime configuration persitance in the internal flash file system
- Interrupt-based force and angle evaluation throughout an entire crank revolution
- Dead-spot detection for synchronized power updates4
- Support for two load cells to derive individual power readings and to obtain real pedal power balance (right configuration hasn't been tested yet)
- Support for up to 3 parallel connected BLE devices
- BLE-controlled zero-offset compensation (according to BLE CPS spec., works with Garmin devices)
- BLE-controlled custom force/weight calibration and runtime configuration (using custom svc. & char.)
- Runtime power averaging (exponential decay and over revolutions)

The interrupt-based force and angle evaluation on every load cell update provides stable power calculation and the ability to derive torque efficiency together with pedal power smoothness. The dead-spots detection feature allows synchronizing the power updates to the buttom dead-spot angle,
and can be used to implement extreme angle detection feature in the future.

## Hardware Requirements

- Nordic nRF52840 based board
- HX711 load cell amplifier module
- MPU-6050 gyro&accelerometer module
- 4x strain gauge
- Instant adhesive (preferably Loctite 496)

To achieve the most accurate power derivation over an entire crank revolution, the HX711 load-cell needs to be overclocked to provide a higher sample rate of up to: ~296+ samples/s.

## Configuration and Monitoring

Configuration and monitoring can be achieved by using the provided custom BLE service '0xcafe'. It offers a custom characteristic for monitoring, where one can be notified about log message updates and requested command results, and a control characteristic where command requests can be written.   

Available commands are:

| Command | Description                              | Parameter |
|---------|------------------------------------------|-----------|
| 'OC'    | Start offset compensation                | `-`       |
| 'SLM'   | Log mode set                             | `-`       |
| 'GCR'   | Get crank radius (CR)                    | `-`       |
| 'SCR'   | Set crank radius (CR)                    | `m or mm` |
| 'GED'   | Get exponential decay (ED)               | `-`       |
| 'SED'   | Set exponential decay (ED)               | `0-1 / %` |
| 'GPAR'  | Get power averaging over revolution (PAR)| `-`       |
| 'SPAR'  | Set power averaging over revolution (PAR)| `0...n`   |
| 'GIPM'  | Get instant power measure (IPM)          | `-`       |
| 'SIPM'  | Set instant power measure (IPM)          | `0 / 1`   |
| 'GCLB'  | Get active calibration values (CLB)      | `-`       |
| 'CLW'   | Calibrate left  using weight             | `kg`      |
| 'CRW'   | Calibrate right using weight             | `kg`      |
| 'CLF'   | Calibrate left  using force              | `N`       |
| 'CRF'   | Calibrate right using force              | `N`       |
| 'CLS'   | Calibration left  set                    | `<value>` |
| 'CRS'   | Calibration right set                    | `<value>` |
| 'CG'    | Calibration get                          | `-`       |
| 'CA'    | Calibration apply                        | `-`       |
| 'CV'    | Calibration verify                       | `-`       |
| 'CP'    | Calibration persist                      | `-`       |
| 'CL'    | Calibration leave                        | `-`       |


## Calibration Procedure
To calibrate the power meter, simply decide the maximum weight or force you want to apply during the calibration procedure, and enter the mode for the desired crank side by sending a constructed command string.: C<L/R><W/F><value of applied weight or force>.
During calibration, the power meter will average raw measured values over a period of 2.5 seconds. Apply the set force/weight to the pedal axis, rotate the crank arm slowly so that it is parallel to the ground, and hold or move it around that position for about 10 seconds.

This is best achieved on a trainer using a specific weight by turning the rear wheel backward to the desired position and holding it there for more than 2.5 seconds. The interval restarts each time, and the maximum value will be retained until calibration is complete. If there are no observed changes at the maximum force point, the calibration is ready to be applied, verified, and persisted.

To apply the calibration, issue the 'CA' command and observe the provided results. If the reflected force is plausible, persist the calibration by issuing the 'CP' command. You can always exit calibration mode by issuing the 'CL' command.

## Project Status and Insperation

The project is still under development and may require additional work to function correctly on specific nRF52840 boards.
However, it presents already a promising and customizable solution for cyclists who want to build their own power meter.

The project was inspired by previous work of [Steve Jarvis](https://imateapot.dev/homemade-power-meter/) and [EdR's YouTube-Channel](https://www.youtube.com/playlist?list=PLGwDuFncb0Ky7Xtqh5f4zIB2kjJFK9wUi)

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
