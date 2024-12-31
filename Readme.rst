

ARM/Raspberry Pico Setup
------------------------

Toolchain:

.. code-block:: bash

   rustup self update
   rustup update stable
   rustup target add thumbv6m-none-eabi

Dependencies:

.. code-block:: bash

   cargo install elf2uf2-rs
   cargo install flip-link
   cargo install cargo-generate


Project Setup

.. code-block:: bash

   cargo-generate gen -g https://github.com/rp-rs/rp2040-project-template.git

AVR/Arduino Setup
-----------------

.. code-block:: bash

   dnf install openssl-devel systemd-devel
   cargo install cargo-generate
   cargo install ravedude

.. code-block:: bash
   
   dnf install avr-gcc avr-libc avrdude
