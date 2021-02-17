% Produced by CVXGEN, 2021-02-16 11:21:46 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: cvxsolve.m.
% Description: Solution file, via cvx, for use with sample.m.
function [vars, status] = cvxsolve(params, settings)
A = params.A;
Af = params.Af;
B = params.B;
Bf = params.Bf;
Q = params.Q;
u_max = params.u_max;
u_min = params.u_min;
x_0 = params.x_0;
x_max_2 = params.x_max_2;
x_max_3 = params.x_max_3;
x_max_4 = params.x_max_4;
x_min_2 = params.x_min_2;
x_min_3 = params.x_min_3;
x_min_4 = params.x_min_4;
if isfield(params, 'x_ss_1')
  x_ss_1 = params.x_ss_1;
elseif isfield(params, 'x_ss')
  x_ss_1 = params.x_ss{1};
else
  error 'could not find x_ss_1'
end
if isfield(params, 'x_ss_2')
  x_ss_2 = params.x_ss_2;
elseif isfield(params, 'x_ss')
  x_ss_2 = params.x_ss{2};
else
  error 'could not find x_ss_2'
end
if isfield(params, 'x_ss_3')
  x_ss_3 = params.x_ss_3;
elseif isfield(params, 'x_ss')
  x_ss_3 = params.x_ss{3};
else
  error 'could not find x_ss_3'
end
if isfield(params, 'x_ss_4')
  x_ss_4 = params.x_ss_4;
elseif isfield(params, 'x_ss')
  x_ss_4 = params.x_ss{4};
else
  error 'could not find x_ss_4'
end
if isfield(params, 'x_ss_5')
  x_ss_5 = params.x_ss_5;
elseif isfield(params, 'x_ss')
  x_ss_5 = params.x_ss{5};
else
  error 'could not find x_ss_5'
end
if isfield(params, 'x_ss_6')
  x_ss_6 = params.x_ss_6;
elseif isfield(params, 'x_ss')
  x_ss_6 = params.x_ss{6};
else
  error 'could not find x_ss_6'
end
if isfield(params, 'x_ss_7')
  x_ss_7 = params.x_ss_7;
elseif isfield(params, 'x_ss')
  x_ss_7 = params.x_ss{7};
else
  error 'could not find x_ss_7'
end
if isfield(params, 'x_ss_8')
  x_ss_8 = params.x_ss_8;
elseif isfield(params, 'x_ss')
  x_ss_8 = params.x_ss{8};
else
  error 'could not find x_ss_8'
end
if isfield(params, 'x_ss_9')
  x_ss_9 = params.x_ss_9;
elseif isfield(params, 'x_ss')
  x_ss_9 = params.x_ss{9};
else
  error 'could not find x_ss_9'
end
if isfield(params, 'x_ss_10')
  x_ss_10 = params.x_ss_10;
elseif isfield(params, 'x_ss')
  x_ss_10 = params.x_ss{10};
else
  error 'could not find x_ss_10'
end
if isfield(params, 'x_ss_11')
  x_ss_11 = params.x_ss_11;
elseif isfield(params, 'x_ss')
  x_ss_11 = params.x_ss{11};
else
  error 'could not find x_ss_11'
end
if isfield(params, 'x_ss_12')
  x_ss_12 = params.x_ss_12;
elseif isfield(params, 'x_ss')
  x_ss_12 = params.x_ss{12};
else
  error 'could not find x_ss_12'
end
if isfield(params, 'x_ss_13')
  x_ss_13 = params.x_ss_13;
elseif isfield(params, 'x_ss')
  x_ss_13 = params.x_ss{13};
else
  error 'could not find x_ss_13'
end
if isfield(params, 'x_ss_14')
  x_ss_14 = params.x_ss_14;
elseif isfield(params, 'x_ss')
  x_ss_14 = params.x_ss{14};
else
  error 'could not find x_ss_14'
end
if isfield(params, 'x_ss_15')
  x_ss_15 = params.x_ss_15;
elseif isfield(params, 'x_ss')
  x_ss_15 = params.x_ss{15};
else
  error 'could not find x_ss_15'
end
if isfield(params, 'x_ss_16')
  x_ss_16 = params.x_ss_16;
elseif isfield(params, 'x_ss')
  x_ss_16 = params.x_ss{16};
else
  error 'could not find x_ss_16'
end
if isfield(params, 'x_ss_17')
  x_ss_17 = params.x_ss_17;
elseif isfield(params, 'x_ss')
  x_ss_17 = params.x_ss{17};
else
  error 'could not find x_ss_17'
end
if isfield(params, 'x_ss_18')
  x_ss_18 = params.x_ss_18;
elseif isfield(params, 'x_ss')
  x_ss_18 = params.x_ss{18};
else
  error 'could not find x_ss_18'
end
if isfield(params, 'x_ss_19')
  x_ss_19 = params.x_ss_19;
elseif isfield(params, 'x_ss')
  x_ss_19 = params.x_ss{19};
else
  error 'could not find x_ss_19'
end
if isfield(params, 'x_ss_20')
  x_ss_20 = params.x_ss_20;
elseif isfield(params, 'x_ss')
  x_ss_20 = params.x_ss{20};
else
  error 'could not find x_ss_20'
end
if isfield(params, 'x_ss_21')
  x_ss_21 = params.x_ss_21;
elseif isfield(params, 'x_ss')
  x_ss_21 = params.x_ss{21};
else
  error 'could not find x_ss_21'
end
if isfield(params, 'x_ss_22')
  x_ss_22 = params.x_ss_22;
elseif isfield(params, 'x_ss')
  x_ss_22 = params.x_ss{22};
else
  error 'could not find x_ss_22'
end
if isfield(params, 'x_ss_23')
  x_ss_23 = params.x_ss_23;
elseif isfield(params, 'x_ss')
  x_ss_23 = params.x_ss{23};
else
  error 'could not find x_ss_23'
end
if isfield(params, 'x_ss_24')
  x_ss_24 = params.x_ss_24;
elseif isfield(params, 'x_ss')
  x_ss_24 = params.x_ss{24};
else
  error 'could not find x_ss_24'
end
if isfield(params, 'x_ss_25')
  x_ss_25 = params.x_ss_25;
elseif isfield(params, 'x_ss')
  x_ss_25 = params.x_ss{25};
else
  error 'could not find x_ss_25'
end
if isfield(params, 'x_ss_26')
  x_ss_26 = params.x_ss_26;
elseif isfield(params, 'x_ss')
  x_ss_26 = params.x_ss{26};
else
  error 'could not find x_ss_26'
end
if isfield(params, 'x_ss_27')
  x_ss_27 = params.x_ss_27;
elseif isfield(params, 'x_ss')
  x_ss_27 = params.x_ss{27};
else
  error 'could not find x_ss_27'
end
if isfield(params, 'x_ss_28')
  x_ss_28 = params.x_ss_28;
elseif isfield(params, 'x_ss')
  x_ss_28 = params.x_ss{28};
else
  error 'could not find x_ss_28'
end
if isfield(params, 'x_ss_29')
  x_ss_29 = params.x_ss_29;
elseif isfield(params, 'x_ss')
  x_ss_29 = params.x_ss{29};
else
  error 'could not find x_ss_29'
end
if isfield(params, 'x_ss_30')
  x_ss_30 = params.x_ss_30;
elseif isfield(params, 'x_ss')
  x_ss_30 = params.x_ss{30};
else
  error 'could not find x_ss_30'
end
if isfield(params, 'x_ss_31')
  x_ss_31 = params.x_ss_31;
elseif isfield(params, 'x_ss')
  x_ss_31 = params.x_ss{31};
else
  error 'could not find x_ss_31'
end
if isfield(params, 'x_ss_32')
  x_ss_32 = params.x_ss_32;
elseif isfield(params, 'x_ss')
  x_ss_32 = params.x_ss{32};
else
  error 'could not find x_ss_32'
end
if isfield(params, 'x_ss_33')
  x_ss_33 = params.x_ss_33;
elseif isfield(params, 'x_ss')
  x_ss_33 = params.x_ss{33};
else
  error 'could not find x_ss_33'
end
if isfield(params, 'x_ss_34')
  x_ss_34 = params.x_ss_34;
elseif isfield(params, 'x_ss')
  x_ss_34 = params.x_ss{34};
else
  error 'could not find x_ss_34'
end
if isfield(params, 'x_ss_35')
  x_ss_35 = params.x_ss_35;
elseif isfield(params, 'x_ss')
  x_ss_35 = params.x_ss{35};
else
  error 'could not find x_ss_35'
end
if isfield(params, 'x_ss_36')
  x_ss_36 = params.x_ss_36;
elseif isfield(params, 'x_ss')
  x_ss_36 = params.x_ss{36};
else
  error 'could not find x_ss_36'
end
if isfield(params, 'x_ss_37')
  x_ss_37 = params.x_ss_37;
elseif isfield(params, 'x_ss')
  x_ss_37 = params.x_ss{37};
else
  error 'could not find x_ss_37'
end
if isfield(params, 'x_ss_38')
  x_ss_38 = params.x_ss_38;
elseif isfield(params, 'x_ss')
  x_ss_38 = params.x_ss{38};
else
  error 'could not find x_ss_38'
end
if isfield(params, 'x_ss_39')
  x_ss_39 = params.x_ss_39;
elseif isfield(params, 'x_ss')
  x_ss_39 = params.x_ss{39};
else
  error 'could not find x_ss_39'
end
if isfield(params, 'x_ss_40')
  x_ss_40 = params.x_ss_40;
elseif isfield(params, 'x_ss')
  x_ss_40 = params.x_ss{40};
else
  error 'could not find x_ss_40'
end
if isfield(params, 'x_ss_41')
  x_ss_41 = params.x_ss_41;
elseif isfield(params, 'x_ss')
  x_ss_41 = params.x_ss{41};
else
  error 'could not find x_ss_41'
end
if isfield(params, 'x_ss_42')
  x_ss_42 = params.x_ss_42;
elseif isfield(params, 'x_ss')
  x_ss_42 = params.x_ss{42};
else
  error 'could not find x_ss_42'
end
if isfield(params, 'x_ss_43')
  x_ss_43 = params.x_ss_43;
elseif isfield(params, 'x_ss')
  x_ss_43 = params.x_ss{43};
else
  error 'could not find x_ss_43'
end
if isfield(params, 'x_ss_44')
  x_ss_44 = params.x_ss_44;
elseif isfield(params, 'x_ss')
  x_ss_44 = params.x_ss{44};
else
  error 'could not find x_ss_44'
end
if isfield(params, 'x_ss_45')
  x_ss_45 = params.x_ss_45;
elseif isfield(params, 'x_ss')
  x_ss_45 = params.x_ss{45};
else
  error 'could not find x_ss_45'
end
if isfield(params, 'x_ss_46')
  x_ss_46 = params.x_ss_46;
elseif isfield(params, 'x_ss')
  x_ss_46 = params.x_ss{46};
else
  error 'could not find x_ss_46'
end
if isfield(params, 'x_ss_47')
  x_ss_47 = params.x_ss_47;
elseif isfield(params, 'x_ss')
  x_ss_47 = params.x_ss{47};
else
  error 'could not find x_ss_47'
end
if isfield(params, 'x_ss_48')
  x_ss_48 = params.x_ss_48;
elseif isfield(params, 'x_ss')
  x_ss_48 = params.x_ss{48};
else
  error 'could not find x_ss_48'
end
if isfield(params, 'x_ss_49')
  x_ss_49 = params.x_ss_49;
elseif isfield(params, 'x_ss')
  x_ss_49 = params.x_ss{49};
else
  error 'could not find x_ss_49'
end
if isfield(params, 'x_ss_50')
  x_ss_50 = params.x_ss_50;
elseif isfield(params, 'x_ss')
  x_ss_50 = params.x_ss{50};
else
  error 'could not find x_ss_50'
end
if isfield(params, 'x_ss_51')
  x_ss_51 = params.x_ss_51;
elseif isfield(params, 'x_ss')
  x_ss_51 = params.x_ss{51};
else
  error 'could not find x_ss_51'
end
if isfield(params, 'x_ss_52')
  x_ss_52 = params.x_ss_52;
elseif isfield(params, 'x_ss')
  x_ss_52 = params.x_ss{52};
else
  error 'could not find x_ss_52'
end
if isfield(params, 'x_ss_53')
  x_ss_53 = params.x_ss_53;
elseif isfield(params, 'x_ss')
  x_ss_53 = params.x_ss{53};
else
  error 'could not find x_ss_53'
end
if isfield(params, 'x_ss_54')
  x_ss_54 = params.x_ss_54;
elseif isfield(params, 'x_ss')
  x_ss_54 = params.x_ss{54};
else
  error 'could not find x_ss_54'
end
if isfield(params, 'x_ss_55')
  x_ss_55 = params.x_ss_55;
elseif isfield(params, 'x_ss')
  x_ss_55 = params.x_ss{55};
else
  error 'could not find x_ss_55'
end
if isfield(params, 'x_ss_56')
  x_ss_56 = params.x_ss_56;
elseif isfield(params, 'x_ss')
  x_ss_56 = params.x_ss{56};
else
  error 'could not find x_ss_56'
end
if isfield(params, 'x_ss_57')
  x_ss_57 = params.x_ss_57;
elseif isfield(params, 'x_ss')
  x_ss_57 = params.x_ss{57};
else
  error 'could not find x_ss_57'
end
if isfield(params, 'x_ss_58')
  x_ss_58 = params.x_ss_58;
elseif isfield(params, 'x_ss')
  x_ss_58 = params.x_ss{58};
else
  error 'could not find x_ss_58'
end
if isfield(params, 'x_ss_59')
  x_ss_59 = params.x_ss_59;
elseif isfield(params, 'x_ss')
  x_ss_59 = params.x_ss{59};
else
  error 'could not find x_ss_59'
end
if isfield(params, 'x_ss_60')
  x_ss_60 = params.x_ss_60;
elseif isfield(params, 'x_ss')
  x_ss_60 = params.x_ss{60};
else
  error 'could not find x_ss_60'
end
cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable x_1(4, 1);
  variable x_2(4, 1);
  variable x_3(4, 1);
  variable x_4(4, 1);
  variable x_5(4, 1);
  variable x_6(4, 1);
  variable x_7(4, 1);
  variable x_8(4, 1);
  variable x_9(4, 1);
  variable x_10(4, 1);
  variable x_11(4, 1);
  variable x_12(4, 1);
  variable x_13(4, 1);
  variable x_14(4, 1);
  variable x_15(4, 1);
  variable x_16(4, 1);
  variable x_17(4, 1);
  variable x_18(4, 1);
  variable x_19(4, 1);
  variable x_20(4, 1);
  variable x_21(4, 1);
  variable x_22(4, 1);
  variable x_23(4, 1);
  variable x_24(4, 1);
  variable x_25(4, 1);
  variable x_26(4, 1);
  variable x_27(4, 1);
  variable x_28(4, 1);
  variable x_29(4, 1);
  variable x_30(4, 1);
  variable x_31(4, 1);
  variable x_32(4, 1);
  variable x_33(4, 1);
  variable x_34(4, 1);
  variable x_35(4, 1);
  variable x_36(4, 1);
  variable x_37(4, 1);
  variable x_38(4, 1);
  variable x_39(4, 1);
  variable x_40(4, 1);
  variable x_41(4, 1);
  variable x_42(4, 1);
  variable x_43(4, 1);
  variable x_44(4, 1);
  variable x_45(4, 1);
  variable x_46(4, 1);
  variable x_47(4, 1);
  variable x_48(4, 1);
  variable x_49(4, 1);
  variable x_50(4, 1);
  variable x_51(4, 1);
  variable x_52(4, 1);
  variable x_53(4, 1);
  variable x_54(4, 1);
  variable x_55(4, 1);
  variable x_56(4, 1);
  variable x_57(4, 1);
  variable x_58(4, 1);
  variable x_59(4, 1);
  variable x_60(4, 1);
  variable u_0;
  variable u_1;
  variable u_2;
  variable u_3;
  variable u_4;
  variable u_5;
  variable u_6;
  variable u_7;
  variable u_8;
  variable u_9;
  variable u_10;
  variable u_11;
  variable u_12;
  variable u_13;
  variable u_14;
  variable u_15;
  variable u_16;
  variable u_17;
  variable u_18;
  variable u_19;
  variable u_20;
  variable u_21;
  variable u_22;
  variable u_23;
  variable u_24;
  variable u_25;
  variable u_26;
  variable u_27;
  variable u_28;
  variable u_29;
  variable u_30;
  variable u_31;
  variable u_32;
  variable u_33;
  variable u_34;
  variable u_35;
  variable u_36;
  variable u_37;
  variable u_38;
  variable u_39;
  variable u_40;
  variable u_41;
  variable u_42;
  variable u_43;
  variable u_44;
  variable u_45;
  variable u_46;
  variable u_47;
  variable u_48;
  variable u_49;
  variable u_50;
  variable u_51;
  variable u_52;
  variable u_53;
  variable u_54;
  variable u_55;
  variable u_56;
  variable u_57;
  variable u_58;
  variable u_59;

  minimize(quad_form(x_1 - x_ss_1, Q) + quad_form(x_2 - x_ss_2, Q) + quad_form(x_3 - x_ss_3, Q) + quad_form(x_4 - x_ss_4, Q) + quad_form(x_5 - x_ss_5, Q) + quad_form(x_6 - x_ss_6, Q) + quad_form(x_7 - x_ss_7, Q) + quad_form(x_8 - x_ss_8, Q) + quad_form(x_9 - x_ss_9, Q) + quad_form(x_10 - x_ss_10, Q) + quad_form(x_11 - x_ss_11, Q) + quad_form(x_12 - x_ss_12, Q) + quad_form(x_13 - x_ss_13, Q) + quad_form(x_14 - x_ss_14, Q) + quad_form(x_15 - x_ss_15, Q) + quad_form(x_16 - x_ss_16, Q) + quad_form(x_17 - x_ss_17, Q) + quad_form(x_18 - x_ss_18, Q) + quad_form(x_19 - x_ss_19, Q) + quad_form(x_20 - x_ss_20, Q) + quad_form(x_21 - x_ss_21, Q) + quad_form(x_22 - x_ss_22, Q) + quad_form(x_23 - x_ss_23, Q) + quad_form(x_24 - x_ss_24, Q) + quad_form(x_25 - x_ss_25, Q) + quad_form(x_26 - x_ss_26, Q) + quad_form(x_27 - x_ss_27, Q) + quad_form(x_28 - x_ss_28, Q) + quad_form(x_29 - x_ss_29, Q) + quad_form(x_30 - x_ss_30, Q) + quad_form(x_31 - x_ss_31, Q) + quad_form(x_32 - x_ss_32, Q) + quad_form(x_33 - x_ss_33, Q) + quad_form(x_34 - x_ss_34, Q) + quad_form(x_35 - x_ss_35, Q) + quad_form(x_36 - x_ss_36, Q) + quad_form(x_37 - x_ss_37, Q) + quad_form(x_38 - x_ss_38, Q) + quad_form(x_39 - x_ss_39, Q) + quad_form(x_40 - x_ss_40, Q) + quad_form(x_41 - x_ss_41, Q) + quad_form(x_42 - x_ss_42, Q) + quad_form(x_43 - x_ss_43, Q) + quad_form(x_44 - x_ss_44, Q) + quad_form(x_45 - x_ss_45, Q) + quad_form(x_46 - x_ss_46, Q) + quad_form(x_47 - x_ss_47, Q) + quad_form(x_48 - x_ss_48, Q) + quad_form(x_49 - x_ss_49, Q) + quad_form(x_50 - x_ss_50, Q) + quad_form(x_51 - x_ss_51, Q) + quad_form(x_52 - x_ss_52, Q) + quad_form(x_53 - x_ss_53, Q) + quad_form(x_54 - x_ss_54, Q) + quad_form(x_55 - x_ss_55, Q) + quad_form(x_56 - x_ss_56, Q) + quad_form(x_57 - x_ss_57, Q) + quad_form(x_58 - x_ss_58, Q) + quad_form(x_59 - x_ss_59, Q) + quad_form(x_60 - x_ss_60, Q));
  subject to
    x_1 == Af*x_0 + Bf*u_0;
    x_2 == A*x_1 + B*u_1;
    x_3 == A*x_2 + B*u_2;
    x_4 == A*x_3 + B*u_3;
    x_5 == A*x_4 + B*u_4;
    x_6 == A*x_5 + B*u_5;
    x_7 == A*x_6 + B*u_6;
    x_8 == A*x_7 + B*u_7;
    x_9 == A*x_8 + B*u_8;
    x_10 == A*x_9 + B*u_9;
    x_11 == A*x_10 + B*u_10;
    x_12 == A*x_11 + B*u_11;
    x_13 == A*x_12 + B*u_12;
    x_14 == A*x_13 + B*u_13;
    x_15 == A*x_14 + B*u_14;
    x_16 == A*x_15 + B*u_15;
    x_17 == A*x_16 + B*u_16;
    x_18 == A*x_17 + B*u_17;
    x_19 == A*x_18 + B*u_18;
    x_20 == A*x_19 + B*u_19;
    x_21 == A*x_20 + B*u_20;
    x_22 == A*x_21 + B*u_21;
    x_23 == A*x_22 + B*u_22;
    x_24 == A*x_23 + B*u_23;
    x_25 == A*x_24 + B*u_24;
    x_26 == A*x_25 + B*u_25;
    x_27 == A*x_26 + B*u_26;
    x_28 == A*x_27 + B*u_27;
    x_29 == A*x_28 + B*u_28;
    x_30 == A*x_29 + B*u_29;
    x_31 == A*x_30 + B*u_30;
    x_32 == A*x_31 + B*u_31;
    x_33 == A*x_32 + B*u_32;
    x_34 == A*x_33 + B*u_33;
    x_35 == A*x_34 + B*u_34;
    x_36 == A*x_35 + B*u_35;
    x_37 == A*x_36 + B*u_36;
    x_38 == A*x_37 + B*u_37;
    x_39 == A*x_38 + B*u_38;
    x_40 == A*x_39 + B*u_39;
    x_41 == A*x_40 + B*u_40;
    x_42 == A*x_41 + B*u_41;
    x_43 == A*x_42 + B*u_42;
    x_44 == A*x_43 + B*u_43;
    x_45 == A*x_44 + B*u_44;
    x_46 == A*x_45 + B*u_45;
    x_47 == A*x_46 + B*u_46;
    x_48 == A*x_47 + B*u_47;
    x_49 == A*x_48 + B*u_48;
    x_50 == A*x_49 + B*u_49;
    x_51 == A*x_50 + B*u_50;
    x_52 == A*x_51 + B*u_51;
    x_53 == A*x_52 + B*u_52;
    x_54 == A*x_53 + B*u_53;
    x_55 == A*x_54 + B*u_54;
    x_56 == A*x_55 + B*u_55;
    x_57 == A*x_56 + B*u_56;
    x_58 == A*x_57 + B*u_57;
    x_59 == A*x_58 + B*u_58;
    x_60 == A*x_59 + B*u_59;
    x_1(2) <= x_max_2;
    x_2(2) <= x_max_2;
    x_3(2) <= x_max_2;
    x_4(2) <= x_max_2;
    x_5(2) <= x_max_2;
    x_6(2) <= x_max_2;
    x_7(2) <= x_max_2;
    x_8(2) <= x_max_2;
    x_9(2) <= x_max_2;
    x_10(2) <= x_max_2;
    x_11(2) <= x_max_2;
    x_12(2) <= x_max_2;
    x_13(2) <= x_max_2;
    x_14(2) <= x_max_2;
    x_15(2) <= x_max_2;
    x_16(2) <= x_max_2;
    x_17(2) <= x_max_2;
    x_18(2) <= x_max_2;
    x_19(2) <= x_max_2;
    x_20(2) <= x_max_2;
    x_21(2) <= x_max_2;
    x_22(2) <= x_max_2;
    x_23(2) <= x_max_2;
    x_24(2) <= x_max_2;
    x_25(2) <= x_max_2;
    x_26(2) <= x_max_2;
    x_27(2) <= x_max_2;
    x_28(2) <= x_max_2;
    x_29(2) <= x_max_2;
    x_30(2) <= x_max_2;
    x_31(2) <= x_max_2;
    x_32(2) <= x_max_2;
    x_33(2) <= x_max_2;
    x_34(2) <= x_max_2;
    x_35(2) <= x_max_2;
    x_36(2) <= x_max_2;
    x_37(2) <= x_max_2;
    x_38(2) <= x_max_2;
    x_39(2) <= x_max_2;
    x_40(2) <= x_max_2;
    x_41(2) <= x_max_2;
    x_42(2) <= x_max_2;
    x_43(2) <= x_max_2;
    x_44(2) <= x_max_2;
    x_45(2) <= x_max_2;
    x_46(2) <= x_max_2;
    x_47(2) <= x_max_2;
    x_48(2) <= x_max_2;
    x_49(2) <= x_max_2;
    x_50(2) <= x_max_2;
    x_51(2) <= x_max_2;
    x_52(2) <= x_max_2;
    x_53(2) <= x_max_2;
    x_54(2) <= x_max_2;
    x_55(2) <= x_max_2;
    x_56(2) <= x_max_2;
    x_57(2) <= x_max_2;
    x_58(2) <= x_max_2;
    x_59(2) <= x_max_2;
    x_60(2) <= x_max_2;
    x_1(2) >= -x_min_2;
    x_2(2) >= -x_min_2;
    x_3(2) >= -x_min_2;
    x_4(2) >= -x_min_2;
    x_5(2) >= -x_min_2;
    x_6(2) >= -x_min_2;
    x_7(2) >= -x_min_2;
    x_8(2) >= -x_min_2;
    x_9(2) >= -x_min_2;
    x_10(2) >= -x_min_2;
    x_11(2) >= -x_min_2;
    x_12(2) >= -x_min_2;
    x_13(2) >= -x_min_2;
    x_14(2) >= -x_min_2;
    x_15(2) >= -x_min_2;
    x_16(2) >= -x_min_2;
    x_17(2) >= -x_min_2;
    x_18(2) >= -x_min_2;
    x_19(2) >= -x_min_2;
    x_20(2) >= -x_min_2;
    x_21(2) >= -x_min_2;
    x_22(2) >= -x_min_2;
    x_23(2) >= -x_min_2;
    x_24(2) >= -x_min_2;
    x_25(2) >= -x_min_2;
    x_26(2) >= -x_min_2;
    x_27(2) >= -x_min_2;
    x_28(2) >= -x_min_2;
    x_29(2) >= -x_min_2;
    x_30(2) >= -x_min_2;
    x_31(2) >= -x_min_2;
    x_32(2) >= -x_min_2;
    x_33(2) >= -x_min_2;
    x_34(2) >= -x_min_2;
    x_35(2) >= -x_min_2;
    x_36(2) >= -x_min_2;
    x_37(2) >= -x_min_2;
    x_38(2) >= -x_min_2;
    x_39(2) >= -x_min_2;
    x_40(2) >= -x_min_2;
    x_41(2) >= -x_min_2;
    x_42(2) >= -x_min_2;
    x_43(2) >= -x_min_2;
    x_44(2) >= -x_min_2;
    x_45(2) >= -x_min_2;
    x_46(2) >= -x_min_2;
    x_47(2) >= -x_min_2;
    x_48(2) >= -x_min_2;
    x_49(2) >= -x_min_2;
    x_50(2) >= -x_min_2;
    x_51(2) >= -x_min_2;
    x_52(2) >= -x_min_2;
    x_53(2) >= -x_min_2;
    x_54(2) >= -x_min_2;
    x_55(2) >= -x_min_2;
    x_56(2) >= -x_min_2;
    x_57(2) >= -x_min_2;
    x_58(2) >= -x_min_2;
    x_59(2) >= -x_min_2;
    x_60(2) >= -x_min_2;
    x_1(3) <= x_max_3;
    x_2(3) <= x_max_3;
    x_3(3) <= x_max_3;
    x_4(3) <= x_max_3;
    x_5(3) <= x_max_3;
    x_6(3) <= x_max_3;
    x_7(3) <= x_max_3;
    x_8(3) <= x_max_3;
    x_9(3) <= x_max_3;
    x_10(3) <= x_max_3;
    x_11(3) <= x_max_3;
    x_12(3) <= x_max_3;
    x_13(3) <= x_max_3;
    x_14(3) <= x_max_3;
    x_15(3) <= x_max_3;
    x_16(3) <= x_max_3;
    x_17(3) <= x_max_3;
    x_18(3) <= x_max_3;
    x_19(3) <= x_max_3;
    x_20(3) <= x_max_3;
    x_21(3) <= x_max_3;
    x_22(3) <= x_max_3;
    x_23(3) <= x_max_3;
    x_24(3) <= x_max_3;
    x_25(3) <= x_max_3;
    x_26(3) <= x_max_3;
    x_27(3) <= x_max_3;
    x_28(3) <= x_max_3;
    x_29(3) <= x_max_3;
    x_30(3) <= x_max_3;
    x_31(3) <= x_max_3;
    x_32(3) <= x_max_3;
    x_33(3) <= x_max_3;
    x_34(3) <= x_max_3;
    x_35(3) <= x_max_3;
    x_36(3) <= x_max_3;
    x_37(3) <= x_max_3;
    x_38(3) <= x_max_3;
    x_39(3) <= x_max_3;
    x_40(3) <= x_max_3;
    x_41(3) <= x_max_3;
    x_42(3) <= x_max_3;
    x_43(3) <= x_max_3;
    x_44(3) <= x_max_3;
    x_45(3) <= x_max_3;
    x_46(3) <= x_max_3;
    x_47(3) <= x_max_3;
    x_48(3) <= x_max_3;
    x_49(3) <= x_max_3;
    x_50(3) <= x_max_3;
    x_51(3) <= x_max_3;
    x_52(3) <= x_max_3;
    x_53(3) <= x_max_3;
    x_54(3) <= x_max_3;
    x_55(3) <= x_max_3;
    x_56(3) <= x_max_3;
    x_57(3) <= x_max_3;
    x_58(3) <= x_max_3;
    x_59(3) <= x_max_3;
    x_60(3) <= x_max_3;
    x_1(3) >= -x_min_3;
    x_2(3) >= -x_min_3;
    x_3(3) >= -x_min_3;
    x_4(3) >= -x_min_3;
    x_5(3) >= -x_min_3;
    x_6(3) >= -x_min_3;
    x_7(3) >= -x_min_3;
    x_8(3) >= -x_min_3;
    x_9(3) >= -x_min_3;
    x_10(3) >= -x_min_3;
    x_11(3) >= -x_min_3;
    x_12(3) >= -x_min_3;
    x_13(3) >= -x_min_3;
    x_14(3) >= -x_min_3;
    x_15(3) >= -x_min_3;
    x_16(3) >= -x_min_3;
    x_17(3) >= -x_min_3;
    x_18(3) >= -x_min_3;
    x_19(3) >= -x_min_3;
    x_20(3) >= -x_min_3;
    x_21(3) >= -x_min_3;
    x_22(3) >= -x_min_3;
    x_23(3) >= -x_min_3;
    x_24(3) >= -x_min_3;
    x_25(3) >= -x_min_3;
    x_26(3) >= -x_min_3;
    x_27(3) >= -x_min_3;
    x_28(3) >= -x_min_3;
    x_29(3) >= -x_min_3;
    x_30(3) >= -x_min_3;
    x_31(3) >= -x_min_3;
    x_32(3) >= -x_min_3;
    x_33(3) >= -x_min_3;
    x_34(3) >= -x_min_3;
    x_35(3) >= -x_min_3;
    x_36(3) >= -x_min_3;
    x_37(3) >= -x_min_3;
    x_38(3) >= -x_min_3;
    x_39(3) >= -x_min_3;
    x_40(3) >= -x_min_3;
    x_41(3) >= -x_min_3;
    x_42(3) >= -x_min_3;
    x_43(3) >= -x_min_3;
    x_44(3) >= -x_min_3;
    x_45(3) >= -x_min_3;
    x_46(3) >= -x_min_3;
    x_47(3) >= -x_min_3;
    x_48(3) >= -x_min_3;
    x_49(3) >= -x_min_3;
    x_50(3) >= -x_min_3;
    x_51(3) >= -x_min_3;
    x_52(3) >= -x_min_3;
    x_53(3) >= -x_min_3;
    x_54(3) >= -x_min_3;
    x_55(3) >= -x_min_3;
    x_56(3) >= -x_min_3;
    x_57(3) >= -x_min_3;
    x_58(3) >= -x_min_3;
    x_59(3) >= -x_min_3;
    x_60(3) >= -x_min_3;
    x_1(4) <= x_max_4;
    x_2(4) <= x_max_4;
    x_3(4) <= x_max_4;
    x_4(4) <= x_max_4;
    x_5(4) <= x_max_4;
    x_6(4) <= x_max_4;
    x_7(4) <= x_max_4;
    x_8(4) <= x_max_4;
    x_9(4) <= x_max_4;
    x_10(4) <= x_max_4;
    x_11(4) <= x_max_4;
    x_12(4) <= x_max_4;
    x_13(4) <= x_max_4;
    x_14(4) <= x_max_4;
    x_15(4) <= x_max_4;
    x_16(4) <= x_max_4;
    x_17(4) <= x_max_4;
    x_18(4) <= x_max_4;
    x_19(4) <= x_max_4;
    x_20(4) <= x_max_4;
    x_21(4) <= x_max_4;
    x_22(4) <= x_max_4;
    x_23(4) <= x_max_4;
    x_24(4) <= x_max_4;
    x_25(4) <= x_max_4;
    x_26(4) <= x_max_4;
    x_27(4) <= x_max_4;
    x_28(4) <= x_max_4;
    x_29(4) <= x_max_4;
    x_30(4) <= x_max_4;
    x_31(4) <= x_max_4;
    x_32(4) <= x_max_4;
    x_33(4) <= x_max_4;
    x_34(4) <= x_max_4;
    x_35(4) <= x_max_4;
    x_36(4) <= x_max_4;
    x_37(4) <= x_max_4;
    x_38(4) <= x_max_4;
    x_39(4) <= x_max_4;
    x_40(4) <= x_max_4;
    x_41(4) <= x_max_4;
    x_42(4) <= x_max_4;
    x_43(4) <= x_max_4;
    x_44(4) <= x_max_4;
    x_45(4) <= x_max_4;
    x_46(4) <= x_max_4;
    x_47(4) <= x_max_4;
    x_48(4) <= x_max_4;
    x_49(4) <= x_max_4;
    x_50(4) <= x_max_4;
    x_51(4) <= x_max_4;
    x_52(4) <= x_max_4;
    x_53(4) <= x_max_4;
    x_54(4) <= x_max_4;
    x_55(4) <= x_max_4;
    x_56(4) <= x_max_4;
    x_57(4) <= x_max_4;
    x_58(4) <= x_max_4;
    x_59(4) <= x_max_4;
    x_60(4) <= x_max_4;
    x_1(4) >= -x_min_4;
    x_2(4) >= -x_min_4;
    x_3(4) >= -x_min_4;
    x_4(4) >= -x_min_4;
    x_5(4) >= -x_min_4;
    x_6(4) >= -x_min_4;
    x_7(4) >= -x_min_4;
    x_8(4) >= -x_min_4;
    x_9(4) >= -x_min_4;
    x_10(4) >= -x_min_4;
    x_11(4) >= -x_min_4;
    x_12(4) >= -x_min_4;
    x_13(4) >= -x_min_4;
    x_14(4) >= -x_min_4;
    x_15(4) >= -x_min_4;
    x_16(4) >= -x_min_4;
    x_17(4) >= -x_min_4;
    x_18(4) >= -x_min_4;
    x_19(4) >= -x_min_4;
    x_20(4) >= -x_min_4;
    x_21(4) >= -x_min_4;
    x_22(4) >= -x_min_4;
    x_23(4) >= -x_min_4;
    x_24(4) >= -x_min_4;
    x_25(4) >= -x_min_4;
    x_26(4) >= -x_min_4;
    x_27(4) >= -x_min_4;
    x_28(4) >= -x_min_4;
    x_29(4) >= -x_min_4;
    x_30(4) >= -x_min_4;
    x_31(4) >= -x_min_4;
    x_32(4) >= -x_min_4;
    x_33(4) >= -x_min_4;
    x_34(4) >= -x_min_4;
    x_35(4) >= -x_min_4;
    x_36(4) >= -x_min_4;
    x_37(4) >= -x_min_4;
    x_38(4) >= -x_min_4;
    x_39(4) >= -x_min_4;
    x_40(4) >= -x_min_4;
    x_41(4) >= -x_min_4;
    x_42(4) >= -x_min_4;
    x_43(4) >= -x_min_4;
    x_44(4) >= -x_min_4;
    x_45(4) >= -x_min_4;
    x_46(4) >= -x_min_4;
    x_47(4) >= -x_min_4;
    x_48(4) >= -x_min_4;
    x_49(4) >= -x_min_4;
    x_50(4) >= -x_min_4;
    x_51(4) >= -x_min_4;
    x_52(4) >= -x_min_4;
    x_53(4) >= -x_min_4;
    x_54(4) >= -x_min_4;
    x_55(4) >= -x_min_4;
    x_56(4) >= -x_min_4;
    x_57(4) >= -x_min_4;
    x_58(4) >= -x_min_4;
    x_59(4) >= -x_min_4;
    x_60(4) >= -x_min_4;
    u_0(1) <= u_max;
    u_1(1) <= u_max;
    u_2(1) <= u_max;
    u_3(1) <= u_max;
    u_4(1) <= u_max;
    u_5(1) <= u_max;
    u_6(1) <= u_max;
    u_7(1) <= u_max;
    u_8(1) <= u_max;
    u_9(1) <= u_max;
    u_10(1) <= u_max;
    u_11(1) <= u_max;
    u_12(1) <= u_max;
    u_13(1) <= u_max;
    u_14(1) <= u_max;
    u_15(1) <= u_max;
    u_16(1) <= u_max;
    u_17(1) <= u_max;
    u_18(1) <= u_max;
    u_19(1) <= u_max;
    u_20(1) <= u_max;
    u_21(1) <= u_max;
    u_22(1) <= u_max;
    u_23(1) <= u_max;
    u_24(1) <= u_max;
    u_25(1) <= u_max;
    u_26(1) <= u_max;
    u_27(1) <= u_max;
    u_28(1) <= u_max;
    u_29(1) <= u_max;
    u_30(1) <= u_max;
    u_31(1) <= u_max;
    u_32(1) <= u_max;
    u_33(1) <= u_max;
    u_34(1) <= u_max;
    u_35(1) <= u_max;
    u_36(1) <= u_max;
    u_37(1) <= u_max;
    u_38(1) <= u_max;
    u_39(1) <= u_max;
    u_40(1) <= u_max;
    u_41(1) <= u_max;
    u_42(1) <= u_max;
    u_43(1) <= u_max;
    u_44(1) <= u_max;
    u_45(1) <= u_max;
    u_46(1) <= u_max;
    u_47(1) <= u_max;
    u_48(1) <= u_max;
    u_49(1) <= u_max;
    u_50(1) <= u_max;
    u_51(1) <= u_max;
    u_52(1) <= u_max;
    u_53(1) <= u_max;
    u_54(1) <= u_max;
    u_55(1) <= u_max;
    u_56(1) <= u_max;
    u_57(1) <= u_max;
    u_58(1) <= u_max;
    u_59(1) <= u_max;
    u_0(1) >= -u_min;
    u_1(1) >= -u_min;
    u_2(1) >= -u_min;
    u_3(1) >= -u_min;
    u_4(1) >= -u_min;
    u_5(1) >= -u_min;
    u_6(1) >= -u_min;
    u_7(1) >= -u_min;
    u_8(1) >= -u_min;
    u_9(1) >= -u_min;
    u_10(1) >= -u_min;
    u_11(1) >= -u_min;
    u_12(1) >= -u_min;
    u_13(1) >= -u_min;
    u_14(1) >= -u_min;
    u_15(1) >= -u_min;
    u_16(1) >= -u_min;
    u_17(1) >= -u_min;
    u_18(1) >= -u_min;
    u_19(1) >= -u_min;
    u_20(1) >= -u_min;
    u_21(1) >= -u_min;
    u_22(1) >= -u_min;
    u_23(1) >= -u_min;
    u_24(1) >= -u_min;
    u_25(1) >= -u_min;
    u_26(1) >= -u_min;
    u_27(1) >= -u_min;
    u_28(1) >= -u_min;
    u_29(1) >= -u_min;
    u_30(1) >= -u_min;
    u_31(1) >= -u_min;
    u_32(1) >= -u_min;
    u_33(1) >= -u_min;
    u_34(1) >= -u_min;
    u_35(1) >= -u_min;
    u_36(1) >= -u_min;
    u_37(1) >= -u_min;
    u_38(1) >= -u_min;
    u_39(1) >= -u_min;
    u_40(1) >= -u_min;
    u_41(1) >= -u_min;
    u_42(1) >= -u_min;
    u_43(1) >= -u_min;
    u_44(1) >= -u_min;
    u_45(1) >= -u_min;
    u_46(1) >= -u_min;
    u_47(1) >= -u_min;
    u_48(1) >= -u_min;
    u_49(1) >= -u_min;
    u_50(1) >= -u_min;
    u_51(1) >= -u_min;
    u_52(1) >= -u_min;
    u_53(1) >= -u_min;
    u_54(1) >= -u_min;
    u_55(1) >= -u_min;
    u_56(1) >= -u_min;
    u_57(1) >= -u_min;
    u_58(1) >= -u_min;
    u_59(1) >= -u_min;
cvx_end
vars.u_0 = u_0;
vars.u_1 = u_1;
vars.u{1} = u_1;
vars.u_2 = u_2;
vars.u{2} = u_2;
vars.u_3 = u_3;
vars.u{3} = u_3;
vars.u_4 = u_4;
vars.u{4} = u_4;
vars.u_5 = u_5;
vars.u{5} = u_5;
vars.u_6 = u_6;
vars.u{6} = u_6;
vars.u_7 = u_7;
vars.u{7} = u_7;
vars.u_8 = u_8;
vars.u{8} = u_8;
vars.u_9 = u_9;
vars.u{9} = u_9;
vars.u_10 = u_10;
vars.u{10} = u_10;
vars.u_11 = u_11;
vars.u{11} = u_11;
vars.u_12 = u_12;
vars.u{12} = u_12;
vars.u_13 = u_13;
vars.u{13} = u_13;
vars.u_14 = u_14;
vars.u{14} = u_14;
vars.u_15 = u_15;
vars.u{15} = u_15;
vars.u_16 = u_16;
vars.u{16} = u_16;
vars.u_17 = u_17;
vars.u{17} = u_17;
vars.u_18 = u_18;
vars.u{18} = u_18;
vars.u_19 = u_19;
vars.u{19} = u_19;
vars.u_20 = u_20;
vars.u{20} = u_20;
vars.u_21 = u_21;
vars.u{21} = u_21;
vars.u_22 = u_22;
vars.u{22} = u_22;
vars.u_23 = u_23;
vars.u{23} = u_23;
vars.u_24 = u_24;
vars.u{24} = u_24;
vars.u_25 = u_25;
vars.u{25} = u_25;
vars.u_26 = u_26;
vars.u{26} = u_26;
vars.u_27 = u_27;
vars.u{27} = u_27;
vars.u_28 = u_28;
vars.u{28} = u_28;
vars.u_29 = u_29;
vars.u{29} = u_29;
vars.u_30 = u_30;
vars.u{30} = u_30;
vars.u_31 = u_31;
vars.u{31} = u_31;
vars.u_32 = u_32;
vars.u{32} = u_32;
vars.u_33 = u_33;
vars.u{33} = u_33;
vars.u_34 = u_34;
vars.u{34} = u_34;
vars.u_35 = u_35;
vars.u{35} = u_35;
vars.u_36 = u_36;
vars.u{36} = u_36;
vars.u_37 = u_37;
vars.u{37} = u_37;
vars.u_38 = u_38;
vars.u{38} = u_38;
vars.u_39 = u_39;
vars.u{39} = u_39;
vars.u_40 = u_40;
vars.u{40} = u_40;
vars.u_41 = u_41;
vars.u{41} = u_41;
vars.u_42 = u_42;
vars.u{42} = u_42;
vars.u_43 = u_43;
vars.u{43} = u_43;
vars.u_44 = u_44;
vars.u{44} = u_44;
vars.u_45 = u_45;
vars.u{45} = u_45;
vars.u_46 = u_46;
vars.u{46} = u_46;
vars.u_47 = u_47;
vars.u{47} = u_47;
vars.u_48 = u_48;
vars.u{48} = u_48;
vars.u_49 = u_49;
vars.u{49} = u_49;
vars.u_50 = u_50;
vars.u{50} = u_50;
vars.u_51 = u_51;
vars.u{51} = u_51;
vars.u_52 = u_52;
vars.u{52} = u_52;
vars.u_53 = u_53;
vars.u{53} = u_53;
vars.u_54 = u_54;
vars.u{54} = u_54;
vars.u_55 = u_55;
vars.u{55} = u_55;
vars.u_56 = u_56;
vars.u{56} = u_56;
vars.u_57 = u_57;
vars.u{57} = u_57;
vars.u_58 = u_58;
vars.u{58} = u_58;
vars.u_59 = u_59;
vars.u{59} = u_59;
vars.x_1 = x_1;
vars.x{1} = x_1;
vars.x_2 = x_2;
vars.x{2} = x_2;
vars.x_3 = x_3;
vars.x{3} = x_3;
vars.x_4 = x_4;
vars.x{4} = x_4;
vars.x_5 = x_5;
vars.x{5} = x_5;
vars.x_6 = x_6;
vars.x{6} = x_6;
vars.x_7 = x_7;
vars.x{7} = x_7;
vars.x_8 = x_8;
vars.x{8} = x_8;
vars.x_9 = x_9;
vars.x{9} = x_9;
vars.x_10 = x_10;
vars.x{10} = x_10;
vars.x_11 = x_11;
vars.x{11} = x_11;
vars.x_12 = x_12;
vars.x{12} = x_12;
vars.x_13 = x_13;
vars.x{13} = x_13;
vars.x_14 = x_14;
vars.x{14} = x_14;
vars.x_15 = x_15;
vars.x{15} = x_15;
vars.x_16 = x_16;
vars.x{16} = x_16;
vars.x_17 = x_17;
vars.x{17} = x_17;
vars.x_18 = x_18;
vars.x{18} = x_18;
vars.x_19 = x_19;
vars.x{19} = x_19;
vars.x_20 = x_20;
vars.x{20} = x_20;
vars.x_21 = x_21;
vars.x{21} = x_21;
vars.x_22 = x_22;
vars.x{22} = x_22;
vars.x_23 = x_23;
vars.x{23} = x_23;
vars.x_24 = x_24;
vars.x{24} = x_24;
vars.x_25 = x_25;
vars.x{25} = x_25;
vars.x_26 = x_26;
vars.x{26} = x_26;
vars.x_27 = x_27;
vars.x{27} = x_27;
vars.x_28 = x_28;
vars.x{28} = x_28;
vars.x_29 = x_29;
vars.x{29} = x_29;
vars.x_30 = x_30;
vars.x{30} = x_30;
vars.x_31 = x_31;
vars.x{31} = x_31;
vars.x_32 = x_32;
vars.x{32} = x_32;
vars.x_33 = x_33;
vars.x{33} = x_33;
vars.x_34 = x_34;
vars.x{34} = x_34;
vars.x_35 = x_35;
vars.x{35} = x_35;
vars.x_36 = x_36;
vars.x{36} = x_36;
vars.x_37 = x_37;
vars.x{37} = x_37;
vars.x_38 = x_38;
vars.x{38} = x_38;
vars.x_39 = x_39;
vars.x{39} = x_39;
vars.x_40 = x_40;
vars.x{40} = x_40;
vars.x_41 = x_41;
vars.x{41} = x_41;
vars.x_42 = x_42;
vars.x{42} = x_42;
vars.x_43 = x_43;
vars.x{43} = x_43;
vars.x_44 = x_44;
vars.x{44} = x_44;
vars.x_45 = x_45;
vars.x{45} = x_45;
vars.x_46 = x_46;
vars.x{46} = x_46;
vars.x_47 = x_47;
vars.x{47} = x_47;
vars.x_48 = x_48;
vars.x{48} = x_48;
vars.x_49 = x_49;
vars.x{49} = x_49;
vars.x_50 = x_50;
vars.x{50} = x_50;
vars.x_51 = x_51;
vars.x{51} = x_51;
vars.x_52 = x_52;
vars.x{52} = x_52;
vars.x_53 = x_53;
vars.x{53} = x_53;
vars.x_54 = x_54;
vars.x{54} = x_54;
vars.x_55 = x_55;
vars.x{55} = x_55;
vars.x_56 = x_56;
vars.x{56} = x_56;
vars.x_57 = x_57;
vars.x{57} = x_57;
vars.x_58 = x_58;
vars.x{58} = x_58;
vars.x_59 = x_59;
vars.x{59} = x_59;
vars.x_60 = x_60;
vars.x{60} = x_60;
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
