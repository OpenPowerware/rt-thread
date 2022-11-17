

.section  .text 

.global	 ifr_unset

ifr_unset:
csrc 0x344,a0  //IFR &= ~a0 , 
 ret


	

