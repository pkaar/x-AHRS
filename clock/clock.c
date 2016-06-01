/*******************************************************************************
 * clock.c
 *
 * Description:
 *  This module is used to initialize Kinetis KL-series clocks as described
 *  below.
 *
 * Functions:
 *  void clock_init(void)       initialize multipurpose clock generator
 *  void clock_output(void)     set port C, pin 3 as Flash clock output
 *
 * Note:
 *  This library is intended to be used for Kinetis MKL25Z4 microcontrollers.
 *
 * History:
 *  pka, 31/AUG/2014, initial code
 ******************************************************************************/

#include "clock.h"

#include "mkl25z128xxx4.h"

/*******************************************************************************
 * void clock_init(void)
 *
 * Description:
 *  Initialize multipurpose clock generator (MCG) in PLL engaged external (PEE)
 *  mode. In PEE mode, the MCGOUTCLK is derived from the output of PLL which is
 *  controlled by an external reference clock. The FLL is disabled in a
 *  low-power state.
 *  In this case the MCGOUTCLK is set to 48 MHz which corresponds to the
 *  maximum possible clock frequency for MKL25Z128VLK4. System clock dividers
 *  OUTDIV1 and OUTDIV4 are set in order to reach the following clock
 *  frequencies:
 *      - Core clock    48 MHz (OUTDIV1 = 1)
 *      - System clock  48 MHz (OUTDIV1 = 1)
 *      - Bus clock     24 MHz (OUTDIV4 = 2)
 *      - Flash clock   24 MHz (OUTDIV4 = 2)
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 * Example:
 *  clock_init();
 *  Initialize MCGOUTCLK with a frequency of 48 MHz.
 *
 * Notes:
 *  MCGOUTCLK frequency depends on external crystal oscillator. In this case
 *  a crystal oscillator with a frequency of 8 MHz is used.
 *  When changing the crystal oscillator or adapting MCGOUTCLK divide factors
 *  (OUTDIV1 and OUTDIV4) make sure that all resulting clocks remain in valid
 *  ranges. Exceeding these ranges may result in unpredicted error cases.
 *
 * History:
 *  pka, 31/AUG/2014, initial code
*******************************************************************************/
void clock_init(void)
{
    /* Set system clock dividers. The configuration MUST be done before changing
     * MCGOUTCLK to ensure that all clocks remain in valid ranges (see reference
     * manual p. 117-118). */
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV4(1);

    /* Select frequency range of crystal oscillator and clock source for
     * external reference clock (see reference manual p. 373-374). High
     * frequency range MUST be selected, since oscillator frequency is 8 MHz.
     * The used oscillator is a crystal oscillator. */
    MCG_C2 |= MCG_C2_RANGE0(1) | MCG_C2_EREFS0_MASK;

	/* Select clock source for MCGOUTCLK and set reference divider (see
	 * reference manual p. 372). Select external reference clock as system
	 * clock source and select reference divide factor. The factor MUST be 256
	 * to achieve that the resulting frequency is in range of 31.25-39.0625 kHz.
	 * To select external reference clock, MCG_C1[IREFS] bit MUST be cleared. */
	MCG_C1 |= MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);
	MCG_C1 &= ~(MCG_C1_IREFS_MASK);

	/* Wait for MCG_S[OSCINIT0] bit to be set, indicating that the crystal has
	 * been initialized (see reference manual p. 396)*/
	while (!(MCG_S & MCG_S_OSCINIT0_MASK));

	/* Wait for MCG_S[IREFST] bit to be cleared, indicating that the FLL
	 * reference clock source is the external reference clock (see reference
	 * manual p. 396). */
	while (MCG_S & MCG_S_IREFST_MASK);

	/* Wait for MCG_S[CLKST] bits to be changed to 0x2 (0b10), indicating that
	 * the external reference clock has been appropriately selected (see
	 * reference manual p. 396). */
	while (!(((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x2));

	/* Configure PLL reference frequency (see reference manual p. 376-377).
	 * Setting PLL reference divide factor to 4, results in a PLL reference
	 * frequency of 2 MHz, since the external crystal oscillator runs at a
	 * frequency of 8 MHz. */
	MCG_C5 |= MCG_C5_PRDIV0(3);

	/* Select PLL output as MCG source and set voltage controlled oscillator
	 * (VCO) divider (see reference manual p. 377-378). The VCO divider
	 * establishes the multiplication factor applied to the PLL reference clock
	 * frequency. A multiplication factor of 24 results in a MCGOUTCLK frequency
	 * of 48 MHz, since PLL reference frequency is 2 MHz.*/
	MCG_C6 |= MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0);

	/* Wait for MCG_S[PLLST] bit to be set, indicating that the current source
	 * for the PLLS clock is the PLL (see reference manual p. 396). */
	while (!(MCG_S & MCG_S_PLLST_MASK));

	/* Wait for MCG_S[LOCK0] bit to be set, indicating that the PLL has acquired
	 * lock (see reference manual p. 396). */
	while (!(MCG_S & MCG_S_LOCK0_MASK));

	/* Select PLL as MCGOUTCLK source (see reference manual p. 372). */
	MCG_C1 &= ~MCG_C1_CLKS_MASK;

	/* Wait for MCG_S[CLKST] bits to be changed to 0x3 (0b11), indicating that
	 * the PLL output is selected to feed MCGOUTCLK. */
	while (!(((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x3));
}


/*******************************************************************************
 * void clock_output(void)
 *
 * Description:
 *  Initialize CLKOUT function on PTC3. Flash memory clock is used as output
 *  option for CLKOUT.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 * Example:
 *  clock_output();
 *  Initializes PTC3 as output for Flash memory clock.
 *
 * Notes:
 *  CLKOUT frequency depends on MCGOUTCLK and OUTDIV4. In order to use a
 *  different clock source as OUTCLK, this can be achieved by modifying
 *  SIM_SOPT2[CLKAOUSEL] bits.
 *  In order to avoid errors, the FlexBus module is disabled.
 *
 * History:
 *  pka, 31/AUG/2014, initial code
*******************************************************************************/
void clock_output(void)
{
    /* Enable port C module by enabling its clock. */
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

    /* Select the Flash memory clock output option for the CLKOUT pin. */
    SIM_SOPT2 |= SIM_SOPT2_CLKOUTSEL(2);

    /* Enable CLKOUT function on PTC3 (ALT5 function) and enable high drive
     * strength. */
    PORTC_PCR(3) = PORT_PCR_MUX(5) | PORT_PCR_DSE_MASK;
}
