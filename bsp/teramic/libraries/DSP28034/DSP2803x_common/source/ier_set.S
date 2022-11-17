

.section  .text 

.global	 ier_set

ier_set:
 csrs 0x304,a0  //IER |= a0 
 ret


	

