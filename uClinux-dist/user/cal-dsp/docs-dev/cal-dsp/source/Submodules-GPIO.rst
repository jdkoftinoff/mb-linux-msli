GPIO Submodule
==============


``{RELAY}``:
  Denotes a relay number 1,2,3,4,5 or 6
``{DLI}``:
  Denotes a digital line input 1,2,3,4,5 or 6
``{ADC}``:
  Denotes an A/D converter input 1,2,3 or 4

Parameter Types
---------------


net_address
~~~~~~~~~~~

* String: network address with port, ipv4 or ipv6, IP address or hostname, unicast or multicast address.


smpte_fmt
~~~~~~~~~


int32_t encoding one of the following SMPTE formats:

* 0 : 24 fps
* 1 : 25 fps
* 2 : 30 fps @ 29.97 Hz
* 3 : 30 fps DROP @ 29.97 Hz
* 4 : 30 fps @ 30.00 Hz
* 5 : 30 fps DROP @ 30.00 Hz

smpte_gen_mode
~~~~~~~~~~~~~~

int32_t encoding one of the following SMPTE generator modes:

* 0: inactive
* 1: generate without moving
* 2: generate moving forward


serial_cfg
~~~~~~~~~~

String, serial port configuration in the form::

     <BAUD>,<BITS>,<PARITY>,<STOPBITS>

For example::

     38400,8,N,1


gpio_adc_value
~~~~~~~~~~~~~~

10 Bit A/D converter value::

  Type:        int32_t
  Min:         0
  Max:         1023


Low Level GPIO Addresses
------------------------


Card type
~~~~~~~~~

::

  Address:     /slot/1/type
  Access:      read
  Parameters:  submodule_type, always "GPIO"


Input Audio Channel Count
~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/1/ins
  Access:      read
  Parameters:  int32_t, always 1

Output Audio Channel Count
~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/1/outs
  Access:      read
  Parameters:  int32_t, always 1


D/A Converter Reset
~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/1/GPIO/raw/DAC_RESET
  Access:      r/w
  Parameters:  bool

A/D Converter Reset
~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/1/GPIO/raw/ADC_RESET
  Access:      r/w
  Parameters:  bool


A/D High Pass Filter Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/1/GPIO/raw/ADC_HPF
  Access:      r/w
  Parameters:  bool

Relay Control
~~~~~~~~~~~~~

::

  Address:     /slot/1/GPIO/raw/RELAY_{RELAY}
  Access:      r/w
  Parameters:  bool

DLI Access
~~~~~~~~~~

::

  Address:     /slot/1/GPIO/raw/DLI_{DLI}
  Access:      read
  Parameters:  bool


ADC Access
~~~~~~~~~~

::

  Address:     /slot/1/GPIO/raw/ADC_{ADC}
  Access:      read
  Parameters:  gpio_adc_value



High Level GPIO Addresses
-------------------------

MIDI Port Network Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/MIDI/net
  Access:      r/w
  Parameters:  net_address for TCP server
               net_address for UDP transmission of MTC
               net_address for UDP reception of MTC


RS232 Serial Port Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/RS232/cfg
  Access:      r/w
  Parameters:  serial_cfg


RS232 Serial Port Network Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/RS232/net
  Access:      r/w
  Parameters:  net_address for TCP server


RS232 Serial Port Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/RS422/cfg
  Access:      r/w
  Parameters:  serial_cfg

RS422 Serial Port Network Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/RS422/net
  Access:      r/w
  Parameters:  net_address of TCP server

SMPTE Reader Network
~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/SMPTE/read/net
  Access:      r/w
  Parameters:  net_address for UDP transmission of MTC

SMPTE Reader Lock
~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/SMPTE/read/lock
  Access:      read
  Parameters:  bool


SMPTE Reader Format
~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/SMPTE/read/fmt
  Access:      r/w
  Parameters:  smpte_format


SMPTE Reader Dropout Count
~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/SMPTE/read/drop
  Access:      rw
  Parameters:  int32_t dropout count


SMPTE Reader Freewheel Time
~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/SMPTE/read/freewheel
  Access:      rw
  Parameters:  int32_t freewheel time in ms


SMPTE Reader Current Time
~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/SMPTE/read/time
  Access:      read
  Parameters:  smpte_format
               int64_t gPTP timestamp
               int32_t hours
               int32_t minutes
               int32_t seconds
               int32_t frames
               int32_t subframes


SMPTE Generator Format
~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/SMPTE/gen/fmt
  Access:      r/w
  Parameters:  smpte_format


SMPTE Generator Mode
~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/SMPTE/gen/
  Access:      r/w
  Parameters:  smpte_gen_mode


SMPTE Generator Current Time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /gpio/SMPTE/gen/time
  Access:      r/w
  Parameters:  smpte_format
               int64_t gPTP timestamp
               int32_t hours
               int32_t minutes
               int32_t seconds
               int32_t frames
               int32_t subframes


