

ARM/Raspberry Pico Setup
------------------------

.. code-block:: bash
   
   cargo install elf2uf2-rs
   cargo install flip-link

AVR/Arduino Setup
-----------------

.. code-block:: bash

   dnf install openssl-devel systemd-devel
   cargo install cargo-generate
   cargo install ravedude

.. code-block:: bash
   
   dnf install avr-gcc avr-libc avrdude
