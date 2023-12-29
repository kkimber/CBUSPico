# CBUS library for RaspberryPi PICO

Raspberry PICO-SDK port of the various MERG Arduino libraries:

   * https://github.com/MERG-DEV/CBUSLED
   * https://github.com/MERG-DEV/CBUSSWITCH
   * https://github.com/MERG-DEV/CBUS

and the PICO CAN2040 libraries:

   * https://github.com/obdevel/ACAN2040
   * https://github.com/obdevel/CBUSACAN2040

## Project Status

**Build status**

<img alt="git build action status"
   src="https://github.com/kkimber/CBUSPico/actions/workflows/cmake_rpi_pico.yml/badge.svg"/>

**Static code analysis status**

<a href="https://scan.coverity.com/projects/kkimber-cbuspico">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/29566/badge.svg"/>
</a>

---

**NOTE:** As of December 2023:

This repository currently contains **"work in progress"** code. 

While the code compiles, and is generally functional, but it is currently lacking persisent storage support, so it will not behave as expected.

The repository is public purely to make it easy for MERG members to review changes made to the existing Arduino library code.
