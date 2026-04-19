# Alfa Romeo 166 ICS video extractor/ converter

This project extracts the video signal going to the ICS LCD panel, converting it to composite to be used elsewhere.<br>
<br>
The ICS doesn't self-generate the timing.<br>
Instead, the panel drives the ICS video generator with it own signals: HSYNC, VSYNC and pixel clock.<br>
To get video output without panel, we need to generate the timing signals, this is what the STM32F411 does.<br>
It also generates the composite sync for the RGB to composite video converter.<br>
 
Developed in [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html), also needs [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html).<br>
Programming can be done with [STM32CubeProg](https://www.st.com/en/development-tools/stm32cubeprog.html) (Load the .bin file or compile in the IDE).<br>

### Basic schematic:

<img src="/schematic.jpg">
