function ang = wrapToPiLocal(ang)
%WRAPTOPILOCAL Wrap angles to [-pi, pi] without toolboxes.
ang = mod(ang + pi, 2*pi) - pi;
end