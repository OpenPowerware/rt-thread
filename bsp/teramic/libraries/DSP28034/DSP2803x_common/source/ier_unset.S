

.section  .text 

.global	 ier_unset

ier_unset:
csrc 0x304,a0  //IER &= ~a0 
ret


	

