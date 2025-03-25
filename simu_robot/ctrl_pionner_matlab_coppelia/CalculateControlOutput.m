function [ vr,wr ] = CalculateControlOutput(kb, B, ka, a, ky, y)

vr = kb*B;
wr= ka*a+ky*y;

end