#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "a2d.h"
#include "d2a.h"

//state definitions
#define RESET        0
#define IDLE         1
#define WAIT_TRIGGER 2
#define INT0         3
#define INT0_A       4
#define INT1         5
#define INT2         6
#define INT2_A       7
#define INT2_B       8
#define VSCAN0       9
#define VSCAN1       10
#define VSCAN2       11
#define VSCAN3       12
#define VSCAN4       13
#define VSCAN5       14
#define VSCAN6       15
#define HSCAN0       16
#define HSCAN1       17
#define HSCAN_A2D    18
#define HSCAN2       19

#define IMAGE_MEMADDR	0x30000000						// beginning of the low-power RAM

//function prototypes
uint32_t controller_fsm(uint8_t *);

//declarations
uint32_t integration_time = 1000;
uint32_t default_fsm_wait = 1000;
uint8_t  fsm_state        = RESET;
uint8_t	 *image_array	  = (uint8_t *)IMAGE_MEMADDR;  // memory location for acquired image
uint16_t image_array_size = 6144;					   // size in bytes of an image in memory: (64*64) pixel * 12bit/pixel * byte/8bit
uint8_t  free_running_image_acq = 0;
uint8_t  trigger_image_acq      = 0;
uint8_t  manual_fsm_debugging   = 0;
uint8_t  manual_fsm_advance     = 0;

//-------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------
uint32_t controller_fsm(uint8_t *state)
{
     static uint16_t row_count, col_count, pixel_count, array_count;
	 static uint32_t nxt_state_wait = 1000;			// wait time until next state in micro-seconds
	 uint16_t adc_result;
	 uint16_t dac_CDS_Vbp2;
	     	 
		switch (*state) 
		{
			case RESET:
			   port_pin_set_output_level(ROW_SR_D, 0);
			   port_pin_set_output_level(ROW_SR_CLK, 0);
			   port_pin_set_output_level(ROW_SR_RST, 1);
			   port_pin_set_output_level(ROW_SR_SET, 0);
			   
			   port_pin_set_output_level(RST_SR_D, 0);
			   port_pin_set_output_level(RST_SR_CLK, 0);
			   port_pin_set_output_level(RST_SR_RST, 1);
			   port_pin_set_output_level(RST_SR_SET, 0);
			   
			   port_pin_set_output_level(MUX_SR_D, 0);
			   port_pin_set_output_level(MUX_SR_CLK, 0);
			   port_pin_set_output_level(MUX_SR_RST, 1);
			   port_pin_set_output_level(MUX_SR_SET, 0);
			   
			   port_pin_set_output_level(CDS_PHI1, 0);
			   port_pin_set_output_level(CDS_PHI2, 0);
			   
			   dac_CDS_Vbp2 = 2895;													// added 07/12/17 FC
			   dac_chan_write(&dac_instance, CDS_VBP2, mV_to_int(dac_CDS_Vbp2) );
			   			   			   
			   row_count       = 0;
               col_count       = 0;
			   pixel_count     = 0;
			   array_count     = 0;
		       nxt_state_wait  = default_fsm_wait;
			   *state = IDLE;
			   break;
			   
			case IDLE:  
			   *state = WAIT_TRIGGER;
			   break;
			
			case WAIT_TRIGGER:
			   if (free_running_image_acq == 1)
			      *state = INT0;
			   else {
				   if (trigger_image_acq == 1)
				     *state = INT0;
				   else
				     *state = WAIT_TRIGGER;
			   }
			   break;
			
			case INT0:										// begins acquisition of a new image
			   port_pin_set_output_level(RST_SR_D, 0);
			   port_pin_set_output_level(ROW_SR_D, 1);
			   port_pin_set_output_level(MUX_SR_D, 0);
			   port_pin_set_output_level(RST_SR_CLK, 0);
			   port_pin_set_output_level(ROW_SR_CLK, 0);
			   port_pin_set_output_level(MUX_SR_CLK, 0);
			   port_pin_set_output_level(RST_SR_RST, 0);
			   port_pin_set_output_level(ROW_SR_RST, 0);
			   port_pin_set_output_level(MUX_SR_RST, 0);
			   
			   row_count   = 0;
			   pixel_count = 0;
			   array_count = 0;
		       nxt_state_wait = default_fsm_wait;
			   set_adc_channel(PIXEL_OUT);
			   *state = INT0_A;
			   break;
			
			case INT0_A:
			   port_pin_toggle_output_level(RST_SR_D);
			   col_count = 0;
			   *state = INT1;
			   break;
			
			case INT1:
			   port_pin_set_output_level(RST_SR_CLK, 1);
			   *state = INT2;
			   break;
			
			case INT2:
			   port_pin_set_output_level(RST_SR_CLK, 0);
			   *state = INT2_A;
			   break;
			
			case INT2_A:
			   port_pin_toggle_output_level(RST_SR_D);
			   *state = INT2_B;
			   break;

			case INT2_B:
			   #ifdef XPLAINED_BOARD
			    port_pin_set_output_level(PIN_PB05, 1);
               #endif
			   port_pin_set_output_level(RST_SR_CLK, 1);
			   nxt_state_wait = integration_time;
			   *state = VSCAN0;
			   break;
			
			case VSCAN0:
			   #ifdef XPLAINED_BOARD
			    port_pin_set_output_level(PIN_PB05, 0);
			   #endif 
			   port_pin_set_output_level(RST_SR_CLK, 0);
			   port_pin_set_output_level(ROW_SR_CLK, 1);
			   nxt_state_wait = default_fsm_wait;
			   *state = VSCAN1;
			   break;
			   
			case VSCAN1:
			   port_pin_set_output_level(CDS_PHI1, 1);
			   port_pin_toggle_output_level(RST_SR_D);
			   *state = VSCAN2;
			   break;   

			case VSCAN2:
			   port_pin_set_output_level(CDS_PHI1, 0);
			   *state = VSCAN3;
			   break;			
			   
			case VSCAN3:
			   port_pin_set_output_level(RST_SR_CLK, 1);
			   *state = VSCAN4;
			   break;
			   
			case VSCAN4:
			   port_pin_set_output_level(CDS_PHI2, 1);
			   port_pin_set_output_level(RST_SR_CLK, 0);
			   *state = VSCAN5;
			   break;
			   
			case VSCAN5:
			   port_pin_set_output_level(CDS_PHI2, 0);
			   port_pin_set_output_level(ROW_SR_CLK, 0);
			   port_pin_set_output_level(ROW_SR_D, 0);
			   *state = HSCAN0;
			   break;   
			   
			case HSCAN0:
			   port_pin_set_output_level(MUX_SR_D, 1);
			   col_count = 0;
			   *state = HSCAN1;
			   break;   
			   
		    case HSCAN1:
			   port_pin_set_output_level(MUX_SR_CLK, 1);
			   *state = HSCAN_A2D;
			   break;
			   
            // image_array[0] |	image_array[1]      | image_array[2] | image_array[3] | image_array[4]      | image_array[5] | 
			// adc[11:4]      | adc[3:0] | adc[3:0] | adc[11:4]      | adc[11:4]      | adc[3:0] | adc[3:0] | adc[11:4]      |
			//       pixel_count = 0     |      pixel_count = 1      |      pixel_count = 2      |      pixel_count = 3      |
			case HSCAN_A2D:
 		       adc_result = pixel_count;  //read_adc();
			   
			   if(pixel_count % 2 == 1) {
				 low_nibble = adc_result & 0x0F;
				 image_array[array_count] = high_nibble << 4 + low_nibble;
				 array_count++;
				 image_array[array_count] = adc_result >> 4;
				 array_count++;
				   
			   } else {
                 image_array[array_count] = adc_result >> 4;
				 high_nibble = adc_result & 0x0F;
				 array_count++;
			   }
			   pixel_count++;
			   
//			   if(pixel_count % 2 == 1) {
//				  image_array[pixel_count]   = (adc_result >> 4) & 0xFF;	// high byte
//				  image_array[pixel_count+1] = (adc_result << 4) & 0xF0;	// last four bits
//				} else {
//			  	  image_array[pixel_count]   = (image_array[pixel_count] & 0xF0) + ((adc_result >> 8) & 0x0F);
//				  image_array[pixel_count+1] = (adc_result & 0xFF);
//				}
//			   pixel_count ++;	
			   *state = HSCAN2;
			   break;
			   
			case HSCAN2:
			   port_pin_set_output_level(MUX_SR_D, 0);
			   port_pin_set_output_level(MUX_SR_CLK, 0);
			   col_count ++;
			   if (col_count >= NUM_COLS) {
			      col_count = 0;
				  *state = VSCAN6;
			   } else {
				  *state = HSCAN1; 
			   }
			   break;
			   
			case VSCAN6:
			   row_count++;
			   if (row_count >= NUM_ROWS) {
				  row_count = 0; 
				  trigger_image_acq = 0;			// clear trigger
				  *state = WAIT_TRIGGER;		    // image fully acquired, wait for trigger to acquire next image
			   } else {
				  *state = INT0_A;					// get next image row
			   }
			
			   break;		                 	   
			   			   			
			default:
			   *state = RESET;
			   break;
			
		} 
	return nxt_state_wait;	
}

#endif