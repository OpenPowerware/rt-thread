

.section  .text 

.global	 enable_interrupt



enable_interrupt:
    li a0, 0x88
    csrs mstatus, a0
	ret


	

