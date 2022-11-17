

.section  .text 

.global	 ifr_set

ifr_set:
csrs 0x344,a0  //IFR |= a0 
 ret


	

