function [ V ] = CalculVitesseRoues(v,w)
%r rayon des roues et l moitié distance entre les roues (rayon cercle
%robot)

vd = (v+lw)/r;
vg= (v-lw)/r;

V=[vd;vg];
end