Device Settings
===============

Parameter Types
---------------


Addresses
---------


Vendor ID
~~~~~~~~~

The address, ``/device/identity/vendor_id`` is used as a read only
value that contains an `int32` containing the vendor's 24 bit OUI code.

Getting device vendor::

  TX: [ "/device/identity/vendor_id" ]
  RX: [ "/device/identity/vendor_id" ,s (vendor) ]


Vendor
~~~~~~

The address, ``/device/identity/vendor`` is used as a read only value that contains the following information:

  * A `string` containing the vendor name in UTF-8

Getting device vendor::

  TX: [ "/device/identity/vendor" ]
  RX: [ "/device/identity/vendor" ,s (vendor) ]

Product
~~~~~~~

The address, ``/device/identity/product`` is used as a read only value that contains the following information:

  * A `string` containing the product name

Getting device product name::

  TX: [ "/device/identity/product" ]
  RX: [ "/device/identity/product" ,s (product) ]


Version
~~~~~~~

The address, ``/device/identity/version`` is used as a read only value that contains the following information:

  * A single `string` or multiple `strings` containing the product's firmware/software/hardware version

Getting device versions::

  TX: [ "/device/identity/version" ]
  RX: [ "/device/identity/version" ,sss (software version) (hardware version) ]


Serial Number
~~~~~~~~~~~~~

The address, ``/device/identity/serial`` is used as a read only value that contains the following information:

  * A `string` containing the product serial number

A serial number is expected to be unique within this vendor/product combination.  If the device does not have a unique serial number, the MAC address of the first ethernet port on the device may be used.

Getting device product serial number::

  TX: [ "/device/identity/serial" ]
  RX: [ "/device/identity/serial" ,s (serial) ]


Device Name
~~~~~~~~~~~

The ``/device/name`` address is used as an end-user configurable name for the device.  This device name will be used to set the bonjour device host name.  This device name may be automatically changed by Bonjour if the requested name is not unique on the network.

Read/write value.

Getting device name::

  TX: [ "/device/name" ]
  RX: [ "/device/name" ,s (value) ]

Setting device name::

  TX: [ "/device/name" ,s (value) ]
  RX: [ "/device/name" ,s (value) ]


System Name
~~~~~~~~~~~

Read/write value.

The system name is the system that this device is a member of.

Getting System Name::

  TX: [ "/device/system" ]
  RX: [ "/device/system" ,s (value) ]

Setting System Name::

  TX: [ "/device/system" ,s (value) ]
  RX: [ "/device/system" ,s (value) ]

 

