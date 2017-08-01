/* Wind sensor design for 3D printing
all units are in mm */

thin = 1;  // thicknes of the tail, ideally minimum thickness that can be achieved with printer
bearing_inside_diameter = 8;
bearing_outside_diameter = 16;
bearing_height = 5;
magnet_inside_diameter = 3;
magnet_outside_diameter = 10;
magnet_height = 2;

thick = 2 * bearing_inside_diameter/2 * cos(45);  // thickness of the main constructor; limited by bearing
//thick = bearing_outside_diameter;

radius_total =60;

radius_tail_inner = 25;
height_tail = 60;

height_mid_section = 3;
overlap_mid_section = 10;


/* define the thin tail sheet of the back */
width_tail = radius_total - radius_tail_inner;
volume_tail = width_tail * height_tail * thin;
CoG_tail = radius_total - width_tail/2;
print_layout = 0;
translation = CoG_tail;
//translate([translation, 0, 0]) 
//cube([width_tail, thin, height_tail], center = true);
translate([0, 0, 50]) 
cube([width_tail, thin, height_tail], center = true);


/* SECOND PART */
/* define the mid section of the back, holding the tail */
volume_mid_section = (radius_tail_inner + overlap_mid_section) * thick * height_mid_section;
CoG_mid_section = (radius_tail_inner + overlap_mid_section) /2;

difference(){
translate([CoG_mid_section, 0, 0])
cube([radius_tail_inner + overlap_mid_section, thick, height_mid_section], center = true);
translate([translation, 0, 0]) 
cube([width_tail, thin, height_tail], center = true);
}

/* TODO: 
this section still needs a cut 
for sliding the separately printed tail sheet into
*/

volume_front_section = volume_tail + volume_mid_section;
CoG_front_section = (CoG_tail * volume_tail + CoG_mid_section * volume_mid_section)/volume_front_section;
height_front_section = (volume_front_section/thick)/(CoG_front_section * 2);
translate([-CoG_front_section, 0, 0])
cube([CoG_front_section * 2, thick, height_front_section], center = true);

/* define the clip 
holding onto the bearing and
holding the magnets */

translate([0, 0, -(magnet_outside_diameter + height_front_section/2)])
//cylinder(h=bearing_height + 1, r1=bearing_outside_diameter/2, r2=0, center=true, $fn=20);
difference(){
   cube([bearing_outside_diameter+6, thick, magnet_outside_diameter + height_front_section + 1 + bearing_height*2], center = true);
   translate([0, 0, -(bearing_height+3)])
   cylinder(h=bearing_height + 1, r1=bearing_outside_diameter/2 + 1, r2=bearing_outside_diameter/2, center=true, $fn=20);
   translate([0, 0, -(2*bearing_height+3)])
   cylinder(h=bearing_height + 1, r1=bearing_outside_diameter/2 - 2, r2=bearing_outside_diameter/2 -2, center=true, $fn=20);
   translate([0, 0, -(2*bearing_height)])
   cube([30, 2, bearing_height*2], center = true);
}

rotate([0, 90, 0])
translate([magnet_outside_diameter/2 + height_front_section + 1, 0, 0])
cylinder(h = 40, r1 = magnet_inside_diameter/2, r2 = magnet_inside_diameter/2, center = true, $fn=20);


