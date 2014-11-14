function [imROI,rX,rY]=GetAffineRegion(im0,m0,V,rX,rY,sigmas)
%get affine region
[F,Lambda]=eig(V);
A=F*diag(sqrt(diag(Lambda))./sigmas');%rotation + size with respect to the kernel

%rotate and translate
sX=max(rX)-min(rX)+1;sY=max(rY)-min(rY)+1;
Z=A*[rX;rY];
rX=Z(1,:)+m0(1);rY=Z(2,:)+m0(2);
imROI(:,:,1)=interp2(double(im0(:,:,1)),rX,rY,'linear');
imROI(:,:,2)=interp2(double(im0(:,:,2)),rX,rY,'linear');
imROI(:,:,3)=interp2(double(im0(:,:,3)),rX,rY,'linear');
%imROI=interp2(double(im0(:,:,1)),rX,rY,'linear');
%imROI=reshape(imROI,sX,sY,3);
%imshow(uint8(imROI));
rX=Z(1,:);
rY=Z(2,:);