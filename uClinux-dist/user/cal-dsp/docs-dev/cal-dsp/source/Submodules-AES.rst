AES Submodule
=============


``{SLOT}``:
  Denotes an I/O module submodule slot 1,2, or 3
``{PAIR}``:
  Denotes a channel pair number 1,2,3, or 4


Parameter Types
---------------

.. todo:: Add descriptions of AES control bits and types


Addresses
---------

Card type
~~~~~~~~~

::

  Address:     /slot/{SLOT}/type
  Access:      read
  Parameters:  submodule_type, always "AES"

Input Audio Channel Count
~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ins
  Access:      read
  Parameters:  int32_t, always 8

Output Audio Channel Count
~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/outs
  Access:      read
  Parameters:  int32_t, always 8


Global Sample Rate Converter Reset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AES/raw/SRC_RST
  Access:      r/w
  Parameters:  bool


MCLK Select
~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AES/raw/MCLK_SEL
  Access:      r/w
  Parameters:  int32_t


TX_LOCK
~~~~~~~

::

  Address:     /slot/{SLOT}/AES/raw/TX_LOCK{PAIR}
  Access:      read
  Parameters:  bool

TX_CLKIN
~~~~~~~~

::

  Address:     /slot/{SLOT}/AES/raw/TX_CLKIN{PAIR}
  Access:      r/w
  Parameters:  int32_t (0-7)

TX_CLKDIV
~~~~~~~~

::

  Address:     /slot/{SLOT}/AES/raw/TX_CLKDIV{PAIR}
  Access:      r/w
  Parameters:  int32_t (0-3)

TX_SRCON
~~~~~~~~

::

  Address:     /slot/{SLOT}/AES/raw/TX_SRCON{PAIR}
  Access:      r/w
  Parameters:  bool

TX_RATIO
~~~~~~~~

::

  Address:     /slot/{SLOT}/AES/raw/TX_RATIO{PAIR}
  Access:      r/w
  Parameters:  int32_t (0-65536)


RX_LOCK
~~~~~~~

::

  Address:     /slot/{SLOT}/AES/raw/TX_LOCK{PAIR}
  Access:      read
  Parameters:  bool


RX_RATIO
~~~~~~~~

::

  Address:     /slot/{SLOT}/AES/raw/RX_RATIO{PAIR}
  Access:      r/w
  Parameters:  int32_t (0-65536)

