#!/bin/bash
# correct the GPIO settings to match the stated values in the i96 bus spec
#
# Pat Beirne <patb@pbeirne> 2021
# 
# this script applies only to the OrangePi-i96, booting from u-boot 2012.04.442

echo "OrangePi-i96 fixup GPIO pins"
echo "Version 1.0"

# assert the i96 GPIO pins into GPIO mode
PORTC_GPIO_MASK=0           # no pins need changing
PORTA_GPIO_MASK=0x7e508000  # rda gpioA 15,20,22,25-30 (i96 gpio group)
PORTB_GPIO_MASK=0           # no pins need changing
PORTD_GPIO_MASK=0xc         # rda gpioD 2,3 (i96 gpioi group)
PORTB_GPIO_MASK_NO_CTSRTS=0x0x300  # assert B 8,9 to reuse uart2_cts,rts

# clear the i96 GPIO pins to special function mode
PORTC_SF_MASK=0xfffffe3f  # clear bits C c6,7,8 (i96 uarts)
PORTA_SF_MASK=0xffff91a0  # clear bits A 0-4,6,9-11,13,14 (i96 i2c, spi, i2s)
PORTB_SF_MASK=0xfffffc3f  # clear bits B 6-9 (i96 i2c, uart)
PORTD_SF_MASK=0xFFFFFFFF  # nothing to change

PORTC_IOMUX=0x11a09008
PORTA_IOMUX=0x11a0900c
PORTB_IOMUX=0x11a09010
PORTD_IOMUX=0x11a09014

DEVMEM=''
if [[ -x /usr/bin/devmem2 ]] 
then
  DEVMEM='/usr/bin/devmem2'
elif [[ -x /usr/local/bin/devmem2 ]] 
then
  DEVMEM='/usr/localbin/devmem2'
elif [[ -x /usr/local/bin/devmem2.py ]] 
then
  DEVMEM='/usr/local/bin/devmem2.py'
else
  echo "devmem2 not installed, no fixup done" 
  return 1
fi

get_word() {
  A=`$DEVMEM $1`
  A=${A#*: }
  echo $A
}

# call or_bits with <announcement> <iomux_address> <or_bitmask>
# to set the bits (change to gpio)

or_bits() {
  echo "===" $1 "==="
  READ_VALUE=`get_word $2`
  WRITE_VALUE=$(($READ_VALUE | $3))
  $DEVMEM $2 w $WRITE_VALUE >> /dev/null
  echo "read:" $READ_VALUE " write:" `printf '0x%x' $WRITE_VALUE` "readback: " `get_word $2`
}

or_bits "PORTC set GPIO" $PORTC_IOMUX $PORTC_GPIO_MASK 
or_bits "PORTA set GPIO" $PORTA_IOMUX $PORTA_GPIO_MASK 
or_bits "PORTB set GPIO" $PORTB_IOMUX $PORTB_GPIO_MASK 
or_bits "PORTD set GPIO" $PORTD_IOMUX $PORTD_GPIO_MASK 

and_bits() {
  echo "===" $1 "==="
  READ_VALUE=`get_word $2`
  WRITE_VALUE=$(($READ_VALUE & $3))
  $DEVMEM $2 w $WRITE_VALUE >> /dev/null
  echo "read:" $READ_VALUE " write:" `printf '0x%x' $WRITE_VALUE` "readback: " `get_word $2`
}

and_bits "PORTC set SPECIAL FUNCTION" $PORTC_IOMUX $PORTC_SF_MASK 
and_bits "PORTA set SPECIAL FUNCTION" $PORTA_IOMUX $PORTA_SF_MASK 
and_bits "PORTB set SPECIAL FUNCTION" $PORTB_IOMUX $PORTB_SF_MASK 

