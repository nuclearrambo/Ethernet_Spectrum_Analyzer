# Ethernet_Spectrum_Analyzer
A simple spectrum analyzer on Tiva launchpad using CMSIS library and LWIP protocol stack. 

## Functioning
Using the on chip ADC of the TM4C1294, samples are acquired. 
After acquiring the samples, the FFT function in the CMSIS library takes care of FFT computations.
LWIP protocol stack handles the communication over the Ethernet. The FFT data is sent over the Ethernet using UDP transport. 
The Python script picks the data and plots it on a graph. The graph updates everytime a new data arrives. 
The FFT data is split into multiple packets on the microcontroller because of the MTU limit. 
