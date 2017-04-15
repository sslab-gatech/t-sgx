	.text
	.globl	springboard
	.align	0x1000
	.type	springboard,@function
springboard:                            # @springboard
	.cfi_startproc
	retq
	.globl	sb_no_tsx.entry
sb_no_tsx.entry:
	jmpq	*%r15
	.globl	sb.entry
sb.entry:
	xend
sb.entry.tsx:
	xbegin	sb.entry.tsx
	jmpq	*%r15
	.globl	sb.entry.ex
sb.entry.ex:
	xbegin	sb.entry.ex
	jmpq	*%r15
.Lfunc_end0:
	.align	0x1000
	.size	springboard, .Lfunc_end0-springboard
	.cfi_endproc


	.section	".note.GNU-stack","",@progbits
