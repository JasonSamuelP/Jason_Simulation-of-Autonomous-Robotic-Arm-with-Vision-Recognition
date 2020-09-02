% Call a function to calculate the area for each color plane
function [BlueArea1,RedArea1,GreenArea1] = MyFunction(b)
BluePixels = imsubtract(b(:,:,3),rgb2gray(b));   % Blue plane subtract with grayscale = Blue pixels
BluePixels  = im2bw(BluePixels ,0.18);           % Blue Pixels to Binary Images  with luminance threshold 0.18            
BP1 = bwareafilt(BluePixels,1);                  % Find 1 Blue Pixel object with largest area
RedPixels = imsubtract(b(:,:,1),rgb2gray(b));    % Red plane subtract with grayscale = Red pixels
RedPixels  = im2bw(RedPixels ,0.18);             
RP1 = bwareafilt(RedPixels,1);
GreenPixels = imsubtract(b(:,:,2),rgb2gray(b));  % Green plane subtract with grayscale = Green pixels
GreenPixels = im2bw(GreenPixels ,0.05);             
GP1 = bwareafilt(GreenPixels,1);    

BlueArea = regionprops(BP1,'Area');              % Calculate the area of the Blue Pixel Object
RedArea = regionprops(RP1,'Area');
GreenArea = regionprops(GP1,'Area');

BlueArea1 = struct2array(BlueArea);              % convert to double
if isempty(BlueArea1)                            % if there is no blue pixels at all, BlueArea1 will return 0 
    BlueArea1 = 0;                               % So it will not return empty value
end
RedArea1 = struct2array(RedArea);
if isempty(RedArea1) % determine if array is empty
    RedArea1 = 0;
end
GreenArea1 = struct2array(GreenArea);
if isempty(GreenArea1) % determine if array is empty
    GreenArea1 = 0;
end
end