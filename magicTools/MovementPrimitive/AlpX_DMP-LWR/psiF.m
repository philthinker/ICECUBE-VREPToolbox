% Alp Burak Pehlivan - A part of M.Sc. Thesis
% http://alpx.io/
function r=psiF(h, c, s, i)
    r=exp(-h(i)*(s-c(i))^2);  % h= 1/(2c^2)
end