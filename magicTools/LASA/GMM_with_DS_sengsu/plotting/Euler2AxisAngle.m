function o = Euler2AxisAngle(angle)


o(1, 1) = sin(angle(1, 1) / 2) * sin(angle(2, 1) / 2) * cos(angle(3, 1) / 2)...
    + cos(angle(1, 1) / 2) * cos(angle(2, 1) / 2) * sin(angle(3, 1) / 2);

o(2, 1) = sin(angle(1, 1) / 2) * cos(angle(2, 1) / 2) * cos(angle(3, 1) / 2) + ...
    cos(angle(1, 1) / 2) * sin(angle(2, 1) / 2) * sin(angle(3, 1) / 2);

o(3, 1) = cos(angle(1, 1) / 2) * sin(angle(2, 1) / 2) * cos(angle(3, 1) / 2) - ...
    sin(angle(1, 1) / 2) * cos(angle(2, 1) / 2) * sin(angle(3, 1) / 2);

o(4, 1) = 2 * acos(cos(angle(1, 1) / 2) * cos(angle(2, 1) / 2) * cos(angle(3, 1) / 2) -...
    sin(angle(1, 1) / 2) * sin(angle(2, 1) / 2) * sin(angle(3, 1) / 2));


