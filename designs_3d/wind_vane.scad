/* Wind sensor design for 3D printing
all units are in mm */

thin = 1;  // thicknes of the tail, ideally minimum thickness that can be achieved with printer
bearing_inside_diameter = 8;
thick = 2 * bearing_inside_diameter/2 * cos(45);  // thickness of the main constructor; limited by bearing

radius_total = 60;

radius_tail_inner = 25;
height_tail = 50;

height_mid_section = 6;
overlap_mid_section = 10;

/* define the thin tail sheet of the back */
width_tail = radius_total - radius_tail_inner;
volume_tail = width_tail * height_tail * thin;
CoG_tail = radius_total - width_tail/2;

translate([CoG_tail, 0, 0]) 
cube([width_tail, thin, height_tail], center = true);

/* define the mid section of the back, holding the tail */
volume_mid_section = (radius_tail_inner + overlap_mid_section) * thick * height_mid_section;
CoG_mid_section = (radius_tail_inner + overlap_mid_section) /2;

translate([CoG_mid_section, 0, 0])
cube([radius_tail_inner + overlap_mid_section, thick, height_mid_section], center = true);

/* TODO: 
this section still needs a cut 
for sliding the separately printed tail sheet into
*/

volume_front_section = volume_tail + volume_mid_section;
CoG_front_section = (CoG_tail * volume_tail + CoG_mid_section * volume_mid_section)/volume_front_section;
height_front_section = (volume_front_section/thick)/(CoG_front_section * 2);
translate([-CoG_front_section, 0, 0])
cube([CoG_front_section * 2, thick, height_front_section], center = true);
