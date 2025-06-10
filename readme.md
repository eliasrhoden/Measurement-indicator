# Measurement indicator measurement unit

This repo contains the code for a Nucleo STM32F303K8 and a pc client to utilize a LVDT test indicator.

Typical high precision digital test indicators use a Linear Varaible Differential Transformer (LVDT), used by for example Mahr, Tesa, etc.

![tesa](https://tesatechnology.com/Images/XL/BD%20photos/41%2044%2048%2050%20Afficheurs%20et%20interfaces/44.30013P1.jpg)

More info regarding the principle of LVDT can be found on [wikipedia](https://en.wikipedia.org/wiki/Linear_variable_differential_transformer) but also this site [rdp-group](https://www.rdpe.com/ex/hiw-lvdt.htm) has a really nice animation of the working principle.

![lvdt-gif](https://www.rdpe.com/images/anim/hiw-lvdt.gif)

The general principle is that we generate a sinusoidal exciting voltage that we feed the measurement probe, and the resulting sense voltage is also a sine wave, where the amplitude is related to the distance moved by the probe.

The firmware in *LvdtMeas* generates the excitation voltage on one of its DAC's and samples the resulting sine wave and fits a sine wave to the sensed signal using leas squares.

The sampled sine wave and least square wave is plotted by the client, and the measurment value printed to the console.

![client](docs/client.png)

For the Nucleo code, STM32CubeIDE v1.18.1 was used.
