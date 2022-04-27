function[noise] = generateColoredNoiseMotion(W,dur,frate,speed,dir)

dt=1/frate;
nT = length(0:dt:dur);
dx = 2*pi/W;
x=-pi:dx:(pi-dx);
[XX,YY] = meshgrid(-x,x);


skirt = spike(1, W);	                            % the amp. spectrum
y = skirt.*exp(1i.*2.*pi.*rand(W));               % noise on the complex frequency plane

speeds = speed(1)/frate + speed(2)/frate*randn(size(XX));
direction = dir(1) + dir(2)*randn(size(XX));
noise=nan(W,W,nT);

for tt=1:nT
    noise(:,:,tt)=(real(ifft2(fftshift(y)))); % compute image domain
    y= y.*exp(YY.*speeds.*sind(direction)*1i + XX.*speeds.*cosd(direction)*1i); % Update for next frame
end

% rescale to [0,1]
noise = noise - min(noise(:));
noise = noise./max(noise(:));