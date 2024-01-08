# Data Connection Application

### Motivation
This application was developed in the context of the [RCOM (Computer Networks)](https://sigarra.up.pt/feup/pt/ucurr_geral.ficha_uc_view?pv_ocorrencia_id=333125) course unit, during the 3<sup>rd</sup> year of the Bachelor's Degree in Software Engineering @ FEUP, in collaboration with [Artur Telo](https://github.com/ArturTelo).

### The application
The application allows for the transmission of files of any size and format from one computer to another, provided they are connected through the serial ports.

### How to use it
After connecting the two computers via the serial port, open a terminal on each of them, on the folder containing the code, and execute the command `make` on both, to compile the code.

On the computer to which you want to send the file, run `./bin/main /dev/ttyS11 rx <file_name.file_format>`. On the other computer, run `./bin/main /dev/ttyS10 tx <file_name.file_format>`.

When the program finishes executing (both terminals return to their regular state), the file should appear on the receiving computer, in the folder containing the code.
