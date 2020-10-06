import numpy as np
import p_fkdh as fk

j=fk.dh_par(90,60,-60)
Tm=fk.dh_kine(j)
ee=fk.el_xyzpos(Tm)
p0,p1,p2,p3,p4,p5=fk.el_pos2base(Tm)
fk.fk_draw2(ee)


