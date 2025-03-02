#include <stdint.h>
#include <stdbool.h>
#include "linux_types.h"

/* Note: PRU nmumber should be defined prior to pru specific headers */
#define PRU1
#include "pru_defs.h"
#include "pru_comm.h"
#include "pru_hal.h"

/* ------- Run with BBG-PRUS-1G & BBG-GPIO-1E device overlays loaded -------- */

// #define GPIO0_START_ADDR 0x44E07000
// #define GPIO1_START_ADDR 0x4804C000
// #define GPIO2_START_ADDR 0x481AC000
// #define GPIO3_START_ADDR 0x481ae000
// #define GPIO_DATAIN (0x138)
// #define GPIO_DATAOUT (0x13C)
// #define GPIO_CLEARDATAOUT (0x190)

/* This field must also be changed in host_main.c */

struct pwm_cmd_l cfg;

static void pwm_setup(void)
{
	u8 i;

	cfg.enmask = 0;
	for (i = 0; i < MAX_PWMS; i++)
		cfg.hilo[i][0] = cfg.hilo[i][1] = PRU_us(200);
}

#if 0
static void bubble_sort_index (void)
{
	uint32_t	val[MAX_SORT_CHS], temp_val;
	uint8_t		i, j;
	uint8_t 	sorted_indexes[MAX_SORT_CHS];

	__R30 |= 0x00000001;	/* PRU1.1 - P8.45 */
	val[0] = 0x32345678;
	val[1] = 0x12445e78;
	val[2] = 0x22345678;
	val[3] = 0x13234d78;
	val[4] = 0x61234678;
	val[5] = 0x234678;
	val[6] = 0x34678;
	val[7] = 0x56d48;

	for (i=0; i<MAX_SORT_CHS; i++)
	{
		sorted_indexes[i] = i;
		for (j=i; j<MAX_SORT_CHS; j++)
		{
			if (val[i] < val[j])
			{
				temp_val = val[i];
				val[i] = val[j];
				val[j] = temp_val;
				sorted_indexes[i] = j;
			}
		}
	}
	__R30 &= 0xfffffffe;	/* PRU1.1 - P8.45 */
}
#endif

static inline u32 read_PIEP_COUNT(void)
{
	return PIEP_COUNT;
}

int main(int argc, char *argv[])
{
	u8 i;
	u32 cnt, next;
	u32 msk, setmsk, clrmsk;
	u32 delta, deltamin, tnext, hi, lo;
	u32 *nextp;
	const u32 *hilop;
    u32 period;
	u32 enmask;	/* enable mask */
	u32 stmask;	/* state mask */
	static u32 next_hi_lo[MAX_PWMS][3];
	static struct cxt cxt;
	uint32_t		lLoops = 200000000;
	/* enable OCP master port */
	PRUCFG_SYSCFG &= ~SYSCFG_STANDBY_INIT;
	PRUCFG_SYSCFG = (PRUCFG_SYSCFG &
			~(SYSCFG_IDLE_MODE_M | SYSCFG_STANDBY_MODE_M)) |
			SYSCFG_IDLE_MODE_NO | SYSCFG_STANDBY_MODE_NO;

	/* our PRU wins arbitration */
	PRUCFG_SPP |=  SPP_PRU1_PAD_HP_EN;
	pwm_setup();

	/* configure timer */
	PIEP_GLOBAL_CFG = GLOBAL_CFG_DEFAULT_INC(1) |
			  GLOBAL_CFG_CMP_INC(1);
	PIEP_CMP_STATUS = CMD_STATUS_CMP_HIT(1); /* clear the interrupt */
        PIEP_CMP_CMP1   = 0x0;
	PIEP_CMP_CFG |= CMP_CFG_CMP_EN(1);
        PIEP_GLOBAL_CFG |= GLOBAL_CFG_CNT_ENABLE;

	/* initialize */
	cnt = read_PIEP_COUNT();

	enmask = cfg.enmask;
	stmask = 0;		/* starting all low */

	clrmsk = 0;
	for (i = 0, msk = 1, nextp = &next_hi_lo[0][0], hilop = &cfg.hilo[0][0];
			i < MAX_PWMS;
			i++, msk <<= 1, nextp += 3, hilop += 2) {
		if ((enmask & msk) == 0) {
			nextp[1] = PRU_us(100);	/* default */
			nextp[2] = PRU_us(100);
			continue;
		}
		nextp[0] = cnt;		/* next */
		nextp[1] = 200000;	/* hi */
        nextp[2] = 208000;	/* lo */
        PWM_CMD->periodhi[i][0] = 408000;
        PWM_CMD->periodhi[i][1] = 180000;
	}
    PWM_CMD->enmask = 0;
	clrmsk = enmask;
	setmsk = 0;
	/* guaranteed to be immediate */
	deltamin = 0;
	next = cnt + deltamin;
    PWM_CMD->magic = PWM_REPLY_MAGIC;

	while (0 < (lLoops--))
	{
        //if(PWM_CMD->magic == PWM_CMD_MAGIC)
        {
			msk = PWM_CMD->enmask;
            for(i=0, nextp = &next_hi_lo[0][0]; i<MAX_PWMS;
                i++, nextp += 3){
                //Enable
                if ((PWM_EN_MASK & (msk&(1U<<i))) && (enmask & (msk&(1U<<i))) == 0) {
        		        enmask |= (msk&(1U<<i));

    				    __R30 |= (msk&(1U<<i));
                        nextp[0] = cnt;	//since we start high, wait this amount

                        // first enable
                        if (enmask == (msk&(1U<<i)))
            			    cnt = read_PIEP_COUNT();
                        deltamin = 0;
                        next = cnt;
                }
                //Disable
        		if ((PWM_EN_MASK & (msk&(1U<<i))) && ((msk & ~(1U<<i)) == 0)) {
        			enmask &= ~(1U<<i);
        			__R30 &= ~(1U<<i);
        		}

                //get and set pwm_vals
                if (PWM_EN_MASK & (msk&(1U<<i))) {

            			//nextp = &next_hi_lo[i * 3];
                		nextp[1] = PWM_CMD->periodhi[i][1];
                        period = PWM_CMD->periodhi[i][0];
                		nextp[2] =period - nextp[1];
                }
                PWM_CMD->hilo_read[i][0] = nextp[0];
                PWM_CMD->hilo_read[i][1] = nextp[1];
            }

			// guaranteed to be immediate
			deltamin = 0;

            PWM_CMD->magic = PWM_REPLY_MAGIC;
		}
        PWM_CMD->enmask_read = enmask;
		/* if nothing is enabled just skip it all */
		if (enmask == 0)
			continue;

		setmsk = 0;
		clrmsk = (u32)-1;
		deltamin = PRU_ms(100); /* (1U << 31) - 1; */
		next = cnt + deltamin;

#define SINGLE_PWM(_i) \
	do { \
		if (enmask & (1U << (_i))) { \
			nextp = &next_hi_lo[(_i)][0]; \
			tnext = nextp[0]; \
			hi = nextp[1]; \
			lo = nextp[2]; \
			/* avoid signed arithmetic */ \
			while (((delta = (tnext - cnt)) & (1U << 31)) != 0) { \
				/* toggle the state */ \
				if (stmask & (1U << (_i))) { \
					stmask &= ~(1U << (_i)); \
					clrmsk &= ~(1U << (_i)); \
					tnext += lo; \
				} else { \
					stmask |= (1U << (_i)); \
					setmsk |= (1U << (_i)); \
					tnext += hi; \
				} \
			} \
			if (delta <= deltamin) { \
				deltamin = delta; \
				next = tnext; \
			} \
			nextp[0] = tnext; \
		} \
	} while (0)

#if MAX_PWMS > 0 && (PWM_EN_MASK & BIT(0))
		SINGLE_PWM(0);
#endif
#if MAX_PWMS > 1 && (PWM_EN_MASK & BIT(1))
		SINGLE_PWM(1);
#endif
#if MAX_PWMS > 2 && (PWM_EN_MASK & BIT(2))
		SINGLE_PWM(2);
#endif
#if MAX_PWMS > 3 && (PWM_EN_MASK & BIT(3))
		SINGLE_PWM(3);
#endif
#if MAX_PWMS > 4 && (PWM_EN_MASK & BIT(4))
		SINGLE_PWM(4);
#endif
#if MAX_PWMS > 5 && (PWM_EN_MASK & BIT(5))
		SINGLE_PWM(5);
#endif
#if MAX_PWMS > 6 && (PWM_EN_MASK & BIT(6))
		SINGLE_PWM(6);
#endif
#if MAX_PWMS > 7 && (PWM_EN_MASK & BIT(7))
		SINGLE_PWM(7);
#endif
#if MAX_PWMS > 8 && (PWM_EN_MASK & BIT(8))
		SINGLE_PWM(8);
#endif
#if MAX_PWMS > 9 && (PWM_EN_MASK & BIT(9))
		SINGLE_PWM(9);
#endif
#if MAX_PWMS > 10 && (PWM_EN_MASK & BIT(10))
		SINGLE_PWM(10);
#endif
#if MAX_PWMS > 11 && (PWM_EN_MASK & BIT(11))
		SINGLE_PWM(11);
#endif
#if MAX_PWMS > 12 && (PWM_EN_MASK & BIT(12))
		SINGLE_PWM(12);
#endif

		/* results in set bits where there are changes */

      __R30 = (__R30 & (clrmsk & 0xfff)) | (setmsk & 0xfff);

		/* loop while nothing changes */
		do {
			cnt = read_PIEP_COUNT();
			if(PWM_CMD->magic == PWM_CMD_MAGIC){
				break;
			}
		} while (((next - cnt) & (1U << 31)) == 0);
	}

	__R31 = 35;	// PRUEVENT_0 on PRU_R31_VEC_VALID
	__halt();

	return 0;
}

