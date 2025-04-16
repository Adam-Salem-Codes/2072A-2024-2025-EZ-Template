#include "main.h"

using namespace jas;

/* void motorspopulate::motorpopulate(std::vector<motors::motordata> motors) {
 motortable = {};
 motortable.assign(motors.begin(), motors.end());
}*/
// list of motors to get temperature

float driveTemp;
float intakeTemp;
float wallmechTemp;

vector<lv_obj_t *> motorboxes{};

lv_obj_t *motorview;
lv_obj_t *mainlabel = lv_label_create(motortemps);

static void gettemp(lv_event_t *e) {
	lv_obj_t *target = lv_event_get_target(e);
	const char *getmotor = (char *)lv_event_get_user_data(e);
	double temp = 0.0;

	lv_obj_set_style_bg_opa(target, (temp - 30) * 5, LV_PART_MAIN);
	printf("Temperature: %f \n", temp);
	printf("Motor: %d \n", *getmotor);
}

lv_event_cb_t getTemp = gettemp;

void tempcheck() {
	// motorbar.assign(Motors.motortable.begin(), Motors.motortable.end());
	static lv_style_t stylemotor;
	lv_style_init(&stylemotor);
	lv_style_set_border_color(&stylemotor, lv_color_hex(0xcfffe9));
	lv_style_set_border_width(&stylemotor, 2);
	lv_style_set_radius(&stylemotor, 8);
	lv_style_set_text_color(&stylemotor, lv_color_hex(0x071808));
	lv_style_set_bg_color(&stylemotor, lv_color_hex(0xcfffe9));
	lv_style_set_bg_opa(&stylemotor, 0);
	lv_style_set_text_font(&stylemotor, &pros_font_dejavu_mono_18);
	lv_style_set_text_align(&stylemotor, LV_TEXT_ALIGN_CENTER);
	lv_style_set_pad_ver(&stylemotor, 6);
	int motorrow = 0;
	char vectorprobe = 0;
	
	lv_obj_add_style(mainlabel, &stylemotor, LV_PART_MAIN);
	// lv_obj_set_style_border_color(mainlabel, lv_color_hex(0x071808),
	// LV_PART_MAIN);
	lv_obj_set_style_pad_all(mainlabel, 6, LV_PART_MAIN);
	lv_obj_set_style_text_color(mainlabel, lv_color_hex(0xcfffe9), LV_PART_MAIN);
	lv_obj_set_style_text_font(mainlabel, &lv_font_montserrat_36, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(mainlabel, 255, LV_PART_MAIN);
	lv_obj_align(mainlabel, LV_ALIGN_TOP_MID, 0, 6);
	lv_label_set_text(mainlabel, "Motor Temperatures");
	// printf("motor port count: {port count: %d}\n", motorbar.size());
}

void tempcheckctrl() {
	while(true) {
		
		pros::delay(10);
	}
}