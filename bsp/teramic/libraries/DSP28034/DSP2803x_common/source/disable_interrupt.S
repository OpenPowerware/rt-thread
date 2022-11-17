

.section  .text 

.global	 disable_interrupt


disable_interrupt:
    li a0, 0x0
    csrw mstatus, a0
	ret


	

