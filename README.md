# New_Build_1K
Added ability to change sleeptime via mqtt (using NodRed). Next
  1: make change in callback to not use hard coded topics, ie not TBOffice/sleeptime but just sleeptime. Use strstr();
  2: Add ability to dial in ADC change value
  3: Add config save  - using spiffs. json file with
    a: hostname
    b:1-wire address's
    c: model number, ie HW tyoe
