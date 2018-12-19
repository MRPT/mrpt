/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
const char* velodyne_default_calib_HDL64E_S3 = R"==(
lasers:
- {dist_correction: 1.4139490000000001, dist_correction_x: 1.4198446999999998, dist_correction_y: 1.4058145,
  focal_distance: 10.5, focal_slope: 1.85, horiz_offset_correction: 0.025999999, laser_id: 0,
  min_intensity: 5, rot_correction: -0.07648247457737148, vert_correction: -0.1261818455292898,
  vert_offset_correction: 0.21569468}
- {dist_correction: 1.5094685, dist_correction_x: 1.5403023, dist_correction_y: 1.5012577999999999,
  focal_distance: 15.0, focal_slope: 1.2, horiz_offset_correction: -0.025999999, laser_id: 1,
  min_intensity: 10, rot_correction: -0.03932070772308096, vert_correction: -0.11953745909742221,
  vert_offset_correction: 0.21520962000000002}
- {dist_correction: 1.4579400999999999, dist_correction_x: 1.4846666, dist_correction_y: 1.4228239,
  focal_distance: 24.0, focal_slope: 1.05, horiz_offset_correction: 0.025999999, laser_id: 2,
  min_intensity: 30, rot_correction: 0.06005350008746038, vert_correction: 0.00356118725906175,
  vert_offset_correction: 0.20631704}
- {dist_correction: 1.5046124, dist_correction_x: 1.5335535999999999, dist_correction_y: 1.5401996,
  focal_distance: 14.5, focal_slope: 1.7, horiz_offset_correction: -0.025999999, laser_id: 3,
  min_intensity: 10, rot_correction: 0.09919490315516696, vert_correction: 0.010306535403103584,
  vert_offset_correction: 0.20583200000000001}
- {dist_correction: 1.4370993, dist_correction_x: 1.4714845, dist_correction_y: 1.4524646,
  focal_distance: 15.0, focal_slope: 0.40000001, horiz_offset_correction: 0.025999999,
  laser_id: 4, min_intensity: 30, rot_correction: -0.00095355016485159, vert_correction: -0.1144967440141401,
  vert_offset_correction: 0.21484217000000003}
- {dist_correction: 1.3536626999999999, dist_correction_x: 1.3909094, dist_correction_y: 1.3700326999999999,
  focal_distance: 17.0, focal_slope: 1.45, horiz_offset_correction: -0.025999999,
  laser_id: 5, min_intensity: 30, rot_correction: 0.03824387099052699, vert_correction: -0.10803612329921503,
  vert_offset_correction: 0.21437181}
- {dist_correction: 1.4117873, dist_correction_x: 1.4350795, dist_correction_y: 1.3829624999999999,
  focal_distance: 10.5, focal_slope: 1.0, horiz_offset_correction: 0.025999999, laser_id: 6,
  min_intensity: 10, rot_correction: -0.015297255529962315, vert_correction: -0.14944538101702426,
  vert_offset_correction: 0.21739968999999998}
- {dist_correction: 1.4269646, dist_correction_x: 1.451161, dist_correction_y: 1.4328421,
  focal_distance: 14.0, focal_slope: 1.3, horiz_offset_correction: -0.025999999, laser_id: 7,
  min_intensity: 10, rot_correction: 0.024255809272700053, vert_correction: -0.14344353231329066,
  vert_offset_correction: 0.21695873}
- {dist_correction: 1.3808284000000002, dist_correction_x: 1.421472, dist_correction_y: 1.3838283000000002,
  focal_distance: 15.0, focal_slope: 0.94999999, horiz_offset_correction: 0.025999999,
  laser_id: 8, min_intensity: 30, rot_correction: 0.07354442571180776, vert_correction: -0.10278016723125998,
  vert_offset_correction: 0.21398966000000003}
- {dist_correction: 1.3644336000000001, dist_correction_x: 1.3931616, dist_correction_y: 1.3852122,
  focal_distance: 12.5, focal_slope: 1.95, horiz_offset_correction: -0.025999999,
  laser_id: 9, min_intensity: 10, rot_correction: 0.11317857886058363, vert_correction: -0.09569590023202451,
  vert_offset_correction: 0.21347521}
- {dist_correction: 1.4102663000000002, dist_correction_x: 1.4163402, dist_correction_y: 1.3592377,
  focal_distance: 12.5, focal_slope: 1.95, horiz_offset_correction: 0.025999999, laser_id: 10,
  min_intensity: 10, rot_correction: 0.05954228616823424, vert_correction: -0.1386345201601863,
  vert_offset_correction: 0.21660597}
- {dist_correction: 1.5019737000000002, dist_correction_x: 1.532475, dist_correction_y: 1.5237663000000001,
  focal_distance: 13.0, focal_slope: 1.9, horiz_offset_correction: -0.025999999, laser_id: 11,
  min_intensity: 10, rot_correction: 0.09906092120980835, vert_correction: -0.13181073657049397,
  vert_offset_correction: 0.21610622}
- {dist_correction: 1.4399905000000002, dist_correction_x: 1.4887962, dist_correction_y: 1.4455237,
  focal_distance: 14.5, focal_slope: 1.7, horiz_offset_correction: 0.025999999, laser_id: 12,
  max_intensity: 235, rot_correction: -0.07574798243226695, vert_correction: -0.054438932963427306,
  vert_offset_correction: 0.21049141}
- {dist_correction: 1.5207593, dist_correction_x: 1.5401453, dist_correction_y: 1.5190082,
  focal_distance: 20.5, focal_slope: 1.25, horiz_offset_correction: -0.025999999,
  laser_id: 13, max_intensity: 240, rot_correction: -0.037047761947550245, vert_correction: -0.048730533448148504,
  vert_offset_correction: 0.21007986}
- {dist_correction: 1.4746239, dist_correction_x: 1.5300989999999999, dist_correction_y: 1.4663734,
  focal_distance: 12.5, focal_slope: 1.9, horiz_offset_correction: 0.025999999, laser_id: 14,
  max_intensity: 245, rot_correction: -0.0895955468906376, vert_correction: -0.08961595328025192,
  vert_offset_correction: 0.21303425}
- {dist_correction: 1.4697629, dist_correction_x: 1.5051735, dist_correction_y: 1.4558601,
  focal_distance: 19.5, focal_slope: 0.94999999, horiz_offset_correction: -0.025999999,
  laser_id: 15, max_intensity: 245, rot_correction: -0.05072966149865084, vert_correction: -0.0839353306800186,
  vert_offset_correction: 0.21262267999999998}
- {dist_correction: 1.4143376, dist_correction_x: 1.4432597000000003, dist_correction_y: 1.4038483,
  focal_distance: 24.0, focal_slope: 0.69999999, horiz_offset_correction: 0.025999999,
  laser_id: 16, max_intensity: 245, rot_correction: -0.001864320564407547, vert_correction: -0.043427018903405855,
  vert_offset_correction: 0.20969770000000001}
- {dist_correction: 1.4815961, dist_correction_x: 1.5173615, dist_correction_y: 1.4871606,
  focal_distance: 14.5, focal_slope: 1.65, horiz_offset_correction: -0.025999999,
  laser_id: 17, rot_correction: 0.03747852840556422, vert_correction: -0.03648801216715539,
  vert_offset_correction: 0.20919794}
- {dist_correction: 1.420331, dist_correction_x: 1.4755219, dist_correction_y: 1.3919118000000001,
  focal_distance: 17.5, focal_slope: 1.4, horiz_offset_correction: 0.025999999, laser_id: 18,
  rot_correction: -0.015194972429011364, vert_correction: -0.07906187922560139, vert_offset_correction: 0.21226994000000002}
- {dist_correction: 1.4900717, dist_correction_x: 1.5143349000000002, dist_correction_y: 1.4829907,
  focal_distance: 24.0, focal_slope: 1.25, horiz_offset_correction: -0.025999999,
  laser_id: 19, rot_correction: 0.02391977928648433, vert_correction: -0.07255814015150577,
  vert_offset_correction: 0.21179958}
- {dist_correction: 1.4823865, dist_correction_x: 1.5357741999999999, dist_correction_y: 1.4956403,
  focal_distance: 24.0, focal_slope: 1.1, horiz_offset_correction: 0.025999999, laser_id: 20,
  rot_correction: 0.07264374331532833, vert_correction: -0.031587736747464255, vert_offset_correction: 0.20884519999999998}
- {dist_correction: 1.5553201, dist_correction_x: 1.5698593, dist_correction_y: 1.5624425,
  focal_distance: 15.5, focal_slope: 1.55, horiz_offset_correction: -0.025999999,
  laser_id: 21, rot_correction: 0.11143265968730214, vert_correction: -0.025460288914769372,
  vert_offset_correction: 0.20840424}
- {dist_correction: 1.4437845999999999, dist_correction_x: 1.4909378000000002, dist_correction_y: 1.4347861,
  focal_distance: 20.5, focal_slope: 1.2, horiz_offset_correction: 0.025999999, laser_id: 22,
  rot_correction: 0.05942554283989759, vert_correction: -0.06665878416276627, vert_offset_correction: 0.21137333000000003}
- {dist_correction: 1.4939513, dist_correction_x: 1.5194868, dist_correction_y: 1.5106346,
  focal_distance: 14.5, focal_slope: 1.65, horiz_offset_correction: -0.025999999,
  laser_id: 23, rot_correction: 0.09797042203986979, vert_correction: -0.06055112836378901,
  vert_offset_correction: 0.21093236999999998}
- {dist_correction: 1.4521244999999998, dist_correction_x: 1.4884882, dist_correction_y: 1.4649333,
  focal_distance: 11.5, focal_slope: 2.0, horiz_offset_correction: 0.025999999, laser_id: 24,
  rot_correction: -0.07658879305408597, vert_correction: 0.015620435355027301, vert_offset_correction: 0.20544985000000002}
- {dist_correction: 1.5351622, dist_correction_x: 1.524973, dist_correction_y: 1.5029565,
  focal_distance: 18.5, focal_slope: 1.25, horiz_offset_correction: -0.025999999,
  laser_id: 25, rot_correction: -0.0379098725675606, vert_correction: 0.022567997346205196,
  vert_offset_correction: 0.20495010000000002}
- {dist_correction: 1.5321327, dist_correction_x: 1.5833431999999998, dist_correction_y: 1.5359726,
  focal_distance: 13.0, focal_slope: 1.9, horiz_offset_correction: 0.025999999, laser_id: 26,
  rot_correction: -0.0904730670226138, vert_correction: -0.019943931467746017, vert_offset_correction: 0.20800737000000002}
- {dist_correction: 1.5166023000000002, dist_correction_x: 1.4874829, dist_correction_y: 1.4625635,
  focal_distance: 21.5, focal_slope: 1.1, horiz_offset_correction: -0.025999999, laser_id: 27,
  rot_correction: -0.051308328902808065, vert_correction: -0.013200099491225388, vert_offset_correction: 0.20752234000000003}
- {dist_correction: 1.4549194, dist_correction_x: 1.4688664, dist_correction_y: 1.4194371000000001,
  focal_distance: 13.5, focal_slope: 1.83, horiz_offset_correction: 0.025999999, laser_id: 28,
  rot_correction: -0.0019143155208309244, vert_correction: 0.027266617424120905, vert_offset_correction: 0.20461203000000003}
- {dist_correction: 1.5617532, dist_correction_x: 1.5887217999999999, dist_correction_y: 1.5643922000000001,
  focal_distance: 18.5, focal_slope: 1.35, horiz_offset_correction: -0.025999999,
  laser_id: 29, rot_correction: 0.03681404490741569, vert_correction: 0.03421014630846329,
  vert_offset_correction: 0.20411228}
- {dist_correction: 1.5132924, dist_correction_x: 1.5661812000000002, dist_correction_y: 1.5102689,
  focal_distance: 18.5, focal_slope: 1.35, horiz_offset_correction: 0.025999999, laser_id: 30,
  rot_correction: -0.015542064486559148, vert_correction: -0.007885903531460533, vert_offset_correction: 0.20714018}
- {dist_correction: 1.5638524, dist_correction_x: 1.5621136000000002, dist_correction_y: 1.5383017,
  focal_distance: 21.5, focal_slope: 1.15, horiz_offset_correction: -0.025999999,
  laser_id: 31, rot_correction: 0.023284423588222334, vert_correction: -0.0015491891506552067,
  vert_offset_correction: 0.20668451000000002}
- {dist_correction: 1.3077792, dist_correction_x: 1.3433452, dist_correction_y: 1.2840973,
  focal_distance: 10.5, focal_slope: 2.0, horiz_offset_correction: 0.025999999, laser_id: 32,
  rot_correction: -0.1278634161583795, vert_correction: -0.39021216204755743, vert_offset_correction: 0.15970787}
- {dist_correction: 1.4303885, dist_correction_x: 1.4429931999999999, dist_correction_y: 1.4607234,
  focal_distance: 0.25, focal_slope: 1.0, horiz_offset_correction: -0.025999999, laser_id: 33,
  rot_correction: -0.066528586091226, vert_correction: -0.38355018793281753, vert_offset_correction: 0.15922519}
- {dist_correction: 1.3746819, dist_correction_x: 1.3873923, dist_correction_y: 1.4089924999999999,
  focal_distance: 0.25, focal_slope: 1.0, horiz_offset_correction: 0.025999999, laser_id: 34,
  rot_correction: 0.08854290740283328, vert_correction: -0.197138848550284, vert_offset_correction: 0.14656122}
- {dist_correction: 1.3814778, dist_correction_x: 1.3932765, dist_correction_y: 1.4301869,
  focal_distance: 11.0, focal_slope: 2.0, horiz_offset_correction: -0.025999999, laser_id: 35,
  rot_correction: 0.14547956884148489, vert_correction: -0.18768579625563228, vert_offset_correction: 0.14595152}
- {dist_correction: 1.4313307, dist_correction_x: 1.4734517, dist_correction_y: 1.4390849000000001,
  focal_distance: 9.5, focal_slope: 1.85, horiz_offset_correction: 0.025999999, laser_id: 36,
  rot_correction: -0.005071588212420635, vert_correction: -0.3750836655445631, vert_offset_correction: 0.15861549}
- {dist_correction: 1.3414835, dist_correction_x: 1.3820609, dist_correction_y: 1.355589,
  focal_distance: 9.5, focal_slope: 1.4, horiz_offset_correction: -0.025999999, laser_id: 37,
  rot_correction: 0.05427589303135225, vert_correction: -0.36762882325722723, vert_offset_correction: 0.15808201}
- {dist_correction: 1.4895827, dist_correction_x: 1.5283238000000001, dist_correction_y: 1.4803529,
  focal_distance: 9.0, focal_slope: 1.65, horiz_offset_correction: 0.025999999, laser_id: 38,
  rot_correction: -0.028143856558087547, vert_correction: -0.4285668719175616, vert_offset_correction: 0.16254044}
- {dist_correction: 1.3727733000000002, dist_correction_x: 1.3623596, dist_correction_y: 1.392661,
  focal_distance: 0.25, focal_slope: 1.15, horiz_offset_correction: -0.025999999,
  laser_id: 39, rot_correction: 0.03172272051181349, vert_correction: -0.4214410765541042,
  vert_offset_correction: 0.16200695}
- {dist_correction: 1.4348983999999998, dist_correction_x: 1.4775407000000003, dist_correction_y: 1.4712174999999998,
  focal_distance: 0.25, focal_slope: 0.92000002, horiz_offset_correction: 0.025999999,
  laser_id: 40, rot_correction: 0.11411698480351568, vert_correction: -0.3565455112681652,
  vert_offset_correction: 0.15729448}
- {dist_correction: 1.335618, dist_correction_x: 1.3519691, dist_correction_y: 1.3315152000000001,
  focal_distance: 11.200000000000001, focal_slope: 2.0, horiz_offset_correction: -0.025999999,
  laser_id: 41, rot_correction: 0.1735498527902839, vert_correction: -0.34717860842539194,
  vert_offset_correction: 0.15663397}
- {dist_correction: 1.4905824, dist_correction_x: 1.488313, dist_correction_y: 1.4948479000000001,
  focal_distance: 0.25, focal_slope: 1.1, horiz_offset_correction: 0.025999999, laser_id: 42,
  rot_correction: 0.09564756333839054, vert_correction: -0.4110102035460227, vert_offset_correction: 0.16123213}
- {dist_correction: 1.3288423, dist_correction_x: 1.3409723, dist_correction_y: 1.3440451,
  focal_distance: 10.0, focal_slope: 1.95, horiz_offset_correction: -0.025999999,
  laser_id: 43, rot_correction: 0.15538288292189534, vert_correction: -0.40083030888437793,
  vert_offset_correction: 0.16048269}
- {dist_correction: 1.3771290999999999, dist_correction_x: 1.3773239000000002, dist_correction_y: 1.3497290000000002,
  focal_distance: 11.5, focal_slope: 2.0, horiz_offset_correction: 0.025999999, laser_id: 44,
  rot_correction: -0.12413605610123538, vert_correction: -0.2840315837972709, vert_offset_correction: 0.15228986}
- {dist_correction: 1.3367807, dist_correction_x: 1.370152, dist_correction_y: 1.370934,
  focal_distance: 0.25, focal_slope: 0.44999999, horiz_offset_correction: -0.025999999,
  laser_id: 45, rot_correction: -0.06413447432303407, vert_correction: -0.27761533458791926,
  vert_offset_correction: 0.15185799}
- {dist_correction: 1.4788651, dist_correction_x: 1.5014308, dist_correction_y: 1.4727454,
  focal_distance: 11.5, focal_slope: 2.0, horiz_offset_correction: 0.025999999, laser_id: 46,
  rot_correction: -0.14872602785785152, vert_correction: -0.3359268721635124, vert_offset_correction: 0.15584644}
- {dist_correction: 1.3220766000000002, dist_correction_x: 1.3654865, dist_correction_y: 1.3739757000000001,
  focal_distance: 0.25, focal_slope: 0.94999999, horiz_offset_correction: -0.025999999,
  laser_id: 47, rot_correction: -0.08707280959256145, vert_correction: -0.33026751989087316,
  vert_offset_correction: 0.15545268}
- {dist_correction: 1.4612433999999999, dist_correction_x: 1.5250436, dist_correction_y: 1.4817635999999998,
  focal_distance: 13.0, focal_slope: 1.0, horiz_offset_correction: 0.025999999, laser_id: 48,
  rot_correction: -0.006799911150716457, vert_correction: -0.26851710773019805, vert_offset_correction: 0.15124829}
- {dist_correction: 1.3407353000000002, dist_correction_x: 1.3478844, dist_correction_y: 1.3154279,
  focal_distance: 13.5, focal_slope: 1.8, horiz_offset_correction: -0.025999999, laser_id: 49,
  rot_correction: 0.05242808851765633, vert_correction: -0.26051854302098837, vert_offset_correction: 0.1507148}
- {dist_correction: 1.3465115, dist_correction_x: 1.3874431999999999, dist_correction_y: 1.3508052000000002,
  focal_distance: 12.0, focal_slope: 1.85, horiz_offset_correction: 0.025999999, laser_id: 50,
  rot_correction: -0.028383715411859876, vert_correction: -0.3216451745070007, vert_offset_correction: 0.15485568000000002}
- {dist_correction: 1.3629696999999998, dist_correction_x: 1.4037315000000001, dist_correction_y: 1.3869237,
  focal_distance: 12.0, focal_slope: 1.1, horiz_offset_correction: -0.025999999, laser_id: 51,
  rot_correction: 0.031843273893907245, vert_correction: -0.3135280670349956, vert_offset_correction: 0.15429679000000002}
- {dist_correction: 1.4472754, dist_correction_x: 1.4817390000000001, dist_correction_y: 1.4782272,
  focal_distance: 2.0, focal_slope: 1.0, horiz_offset_correction: 0.025999999, laser_id: 52,
  rot_correction: 0.10955275158734502, vert_correction: -0.24941678290173272, vert_offset_correction: 0.14997807999999999}
- {dist_correction: 1.3800671, dist_correction_x: 1.3796414, dist_correction_y: 1.3943782,
  focal_distance: 11.200000000000001, focal_slope: 2.0, horiz_offset_correction: -0.025999999,
  laser_id: 53, rot_correction: 0.16876716543828738, vert_correction: -0.2409525119882134,
  vert_offset_correction: 0.14941919}
- {dist_correction: 1.5190169, dist_correction_x: 1.5619051, dist_correction_y: 1.5511705,
  focal_distance: 2.5, focal_slope: 1.05, horiz_offset_correction: 0.025999999, laser_id: 54,
  rot_correction: 0.09037283276367176, vert_correction: -0.30313513748485243, vert_offset_correction: 0.15358547}
- {dist_correction: 1.3803656000000002, dist_correction_x: 1.3960698, dist_correction_y: 1.386106,
  focal_distance: 12.5, focal_slope: 1.9, horiz_offset_correction: -0.025999999, laser_id: 55,
  rot_correction: 0.14871534295217081, vert_correction: -0.29323634555253386, vert_offset_correction: 0.15291226}
- {dist_correction: 1.5827696, dist_correction_x: 1.6050609, dist_correction_y: 1.5844797,
  focal_distance: 10.0, focal_slope: 1.95, horiz_offset_correction: 0.025999999, laser_id: 56,
  rot_correction: -0.12100867217308608, vert_correction: -0.17760463486977288, vert_offset_correction: 0.14530372}
- {dist_correction: 1.3843919, dist_correction_x: 1.3915251000000002, dist_correction_y: 1.4156927,
  focal_distance: 0.25, focal_slope: 0.94999999, horiz_offset_correction: -0.025999999,
  laser_id: 57, rot_correction: -0.06431965899265843, vert_correction: -0.17006926832673774,
  vert_offset_correction: 0.14482104}
- {dist_correction: 1.5491273, dist_correction_x: 1.5298964000000002, dist_correction_y: 1.5107637,
  focal_distance: 11.5, focal_slope: 2.0, horiz_offset_correction: 0.025999999, laser_id: 58,
  rot_correction: -0.14486168214370612, vert_correction: -0.22974121500510264, vert_offset_correction: 0.14868247}
- {dist_correction: 1.3646004, dist_correction_x: 1.3803583000000001, dist_correction_y: 1.4061511,
  focal_distance: 0.25, focal_slope: 1.0, horiz_offset_correction: -0.025999999, laser_id: 59,
  rot_correction: -0.0852480451703211, vert_correction: -0.22391891879349718, vert_offset_correction: 0.14830141}
- {dist_correction: 1.5239935, dist_correction_x: 1.560786, dist_correction_y: 1.5632524,
  focal_distance: 5.0, focal_slope: 1.15, horiz_offset_correction: 0.025999999, laser_id: 60,
  rot_correction: -0.005967226432828876, vert_correction: -0.16231529056824404, vert_offset_correction: 0.14432566}
- {dist_correction: 1.4112029000000001, dist_correction_x: 1.4223886, dist_correction_y: 1.4542918,
  focal_distance: 2.5, focal_slope: 1.05, horiz_offset_correction: -0.025999999, laser_id: 61,
  rot_correction: 0.050600613599087636, vert_correction: -0.15314424682880032, vert_offset_correction: 0.14374136}
- {dist_correction: 1.5013637, dist_correction_x: 1.5208610999999999, dist_correction_y: 1.5006433000000001,
  focal_distance: 16.5, focal_slope: 1.25, horiz_offset_correction: 0.025999999, laser_id: 62,
  rot_correction: -0.027436902217920986, vert_correction: -0.21457119711920336, vert_offset_correction: 0.14769171}
- {dist_correction: 1.4423058, dist_correction_x: 1.4454633000000001, dist_correction_y: 1.4321198000000002,
  focal_distance: 9.0, focal_slope: 1.45, horiz_offset_correction: -0.025999999, laser_id: 63,
  rot_correction: 0.0290479702718764, vert_correction: -0.2079266762969834, vert_offset_correction: 0.14725984}
num_lasers: 64
distance_resolution: 0.002
)==";
