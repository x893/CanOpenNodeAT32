				PRESERVE8
				THUMB

				AREA    |.text|, CODE, READONLY
EnterSystemBoot	PROC
				EXPORT  EnterSystemBoot		[WEAK]
				IMPORT  NormalBoot

				; Load reserved location
				LDR     R0, =0x20000000
				LDR		R1, [R0, #00]
				LDR		R2, [R0, #04]
				STR     R0, [R0, #00]
				STR     R0, [R0, #04]

				; Check first word for signature
				LDR		R3, =0xDEADBEEF
				CMP		R1, R3
				BNE		normal_boot

				; Check second word for signature
				LDR		R3, =0xCAFEBEEF
				CMP		R2, R3
				BNE		normal_boot

				; Start Boot System
				LDR     R0, =0x1FFFAC00
				LDR		R1, [R0, #00]
				MSR		MSP, R1
				LDR		R1, [R0, #04]
				BX		R1

normal_boot		LDR     R0, =NormalBoot
				BX		R0
				ENDP

				ALIGN
				END
