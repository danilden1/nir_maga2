function [angledeg] = getCourceToWaypoint(targ_lat, targ_lon, my_lat, my_lon)
    R = 6372795;
	pi = 3.14159265359;
	cl1 = cos(deg2rad(my_lat));
	cl2 = cos(deg2rad(targ_lat));
	sl1 = sin(deg2rad(my_lat));
	sl2 = sin(deg2rad(targ_lat));
	delta = deg2rad(my_lon) - deg2rad(targ_lon);
	cdelta = cos(delta);
	sdelta = sin(delta);

	y = sqrt(power(cl2 * sdelta, 2) + power(cl1 * sl2 - sl1 * cl2 * cdelta, 2));
	x = sl1 * sl2 + cl1 * cl2 * cdelta;
	ad = atan2(y, x);
	dist = ad * R;

	x = (cl1 * sl2) - (sl1 * cl2 * cdelta);
	y = sdelta * cl2;
	z = atan(-y / x) / pi * 180;
	if x < 0
		z = z + 180.0;
    end
    z2 = fmod((z + 180.0), 360.0) - 180.0;
	z2 = -(z2 / 180.0 * pi);
	anglerad2 = z2 - ((2 * pi) * floor((z2 / (2 * pi))));
	angledeg = (anglerad2 * 180.0) / pi;
	angledeg = 360 - angledeg;
	if angledeg > 359.9
		angledeg = 0.0;
    end
end

