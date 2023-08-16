% Replace 'folder_path' with the actual path to your folder containing the images
folder_path = 'C:\Users\User\Desktop\IVC_labs_starting_point\training_images_residuals';

% List all files in the folder
file_list = dir(fullfile(folder_path, '*'));

% Initialize a cell array to store the names of image files
image_files = {};

% Supported image file extensions (you can add more if needed)
supported_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.gif', '.tif'};
l=0;
% Loop through all files in the folder
for i = 1:numel(file_list)
    [~, ~, ext] = fileparts(file_list(i).name);
    
    % Check if the file extension matches any of the supported image formats
    if any(strcmpi(ext, supported_extensions))
        % Add the file name to the list of image files
        image_files{l+1} = file_list(i).name;
        l=l+1;        
    end
end

for i=1:size(image_files,2)
    img_path = fullfile(folder_path, image_files{i});
    img_data = double(imread(img_path));
    img_cell{i} = img_data;
    y_img_cell{i} = ictRGB2YCbCr(img_data);
    
    imshow(uint8(y_img_cell{i}))

    
    % Display image information (you can do further processing here)
    disp(['Loaded: ', image_files{i}]);
    disp(['Image Size: ', num2str(size(y_img_cell{i}))]);    
end

lower_bound=-6000;
upper_bound=6000;
l=0;

scales = [ 0.07, 0.11, 0.2, 0.4, 0.8, 1.0, 1.5, 2, 3, 4, 4.5]; 
for scaleIdx = 1 : numel(scales)
    qScale   = scales(scaleIdx);
    for i=1:size(y_img_cell,2)
        if mod(i,22)==0 || i==1
            [DC,AC]=IntraEncode_DPCM(img_cell{i}, qScale,63);
            y_I_rec{i}=IntraDecode_DPCM(DC,AC,size(img_cell{i}),qScale,63);
            l=l+1;
        else
            % if i-l==61
            %     keyboard()
            % end
            MV{i-l}=ThreeStepMotionEstimation(y_I_rec{i-1}(:,:,1),y_img_cell{i}(:,:,1));
            mcp{i-l}=ThreeStep_Rec(y_I_rec{i-1},MV{i-l});
            y_residual{i-l}=y_img_cell{i}-mcp{i-l};
            zero_run{i-l}=IntraEncode(y_residual{i-l}, qScale, 64);
            y_residual_dec{i-l}=IntraDecode(zero_run{i-l},size(y_I_rec{1}),qScale,64);
            y_I_rec{i}=y_residual_dec{i-l}+mcp{i-l};
            min_zr(i-l)=min(zero_run{i-l});
            max_zr(i-l)=max(zero_run{i-l});
            min_mv(i-l)=min(MV{i-l}(:));
            max_mv(i-l)=max(MV{i-l}(:));
        end     
    end
    maximum_zr=max(max_zr)
    minimum_zr=min(min_zr)
    maximum_mv=max(max_mv)
    minimum_mv=min(min_mv)
    % lower_bound = min([min(min_AC) lower_bound])% -2000 for adaptive quantization/-1000 for normal
    % upper_bound = max([max(max_AC) upper_bound])% 4000 
    % lower_boundDC = min([min(min_DC) lower_boundDC])%-2000
    % upper_boundDC = max([max(max_DC) upper_boundDC])%200
    pmfZR=0;
    pmfMV=0;
    for i=1:size(zero_run,2)
        cur_pmfZR=stats_marg(zero_run{i},lower_bound:upper_bound);
        pmfZR=pmfZR+cur_pmfZR;
        cur_pmfMV=stats_marg(MV{i}(:),lower_bound:upper_bound);
        pmfMV=pmfMV+cur_pmfMV;
    end
    pmfZR=pmfZR/sum(pmfZR);
    pmfMV=pmfMV/sum(pmfMV);
    [BinaryTreeZR, HuffCodeZR, BinCodeZR, CodelengthsZR] = buildHuffman(pmfZR);
    [BinaryTreeMV, HuffCodeMV, BinCodeMV, CodelengthsMV] = buildHuffman(pmfMV);
    num=num2str(qScale);
    num = strrep(num, '.', '');
    filename=append('Huffman',num);
    filename=append(filename,'.mat');
    filepath=append('huffman_tables/Chapter5ResidualsEOB6000TSS/',filename);
    save(filepath,'BinaryTreeZR','HuffCodeZR','BinCodeZR','CodelengthsZR','BinaryTreeMV','HuffCodeMV','BinCodeMV','CodelengthsMV')
    l=0;
end


%%
function dst = IntraDecode(image, img_size , qScale, block_length)
%  Function Name : IntraDecode.m
%  Input         : image (zero-run encoded image, 1xN)
%                  img_size (original image size)
%                  qScale(quantization scale)
%  Output        : dst   (decoded image)
    num_blocks = img_size ./ 8;
    decoded = ZeroRunDec(image, block_length);
    decoded_blocks= reshape(decoded, [64*num_blocks(1), 3*num_blocks(2)]);    
    inv_zigzag = blockproc(decoded_blocks, [64, 3], @(block_struct) DeZigZag8x8(block_struct.data));
    inv_quant = blockproc(inv_zigzag, [8, 8], @(block_struct) DeQuant8x8(block_struct.data, qScale));
    idct = blockproc(inv_quant, [8, 8], @(block_struct) IDCT8x8(block_struct.data));
    dst = idct;
end

function dst = IntraEncode(image, qScale, block_length)
%  Function Name : IntraEncode.m
%  Input         : image (Original RGB Image)
%                  qScale(quantization scale)
%  Output        : dst   (sequences after zero-run encoding, 1xN)
    I_dct = blockproc(image, [8, 8], @(block_struct) DCT8x8(block_struct.data));   
    quant = blockproc(I_dct, [8, 8], @(block_struct) Quant8x8(block_struct.data, qScale));   
    zigzag = blockproc(quant, [8, 8], @(block_struct) ZigZag8x8(block_struct.data));   
    before_encoding = zigzag(:)';
    dst = ZeroRunEnc(before_encoding, block_length);
end

function dst = IntraDecode_DPCM(DC, AC, img_size , qScale, block_length)
%  Function Name : IntraDecode.m
%  Input         : image (zero-run encoded image, 1xN)
%                  img_size (original image size)
%                  qScale(quantization scale)
%  Output        : dst   (decoded image)
    num_blocks = img_size ./ 8;
    dec_DC = DPCM_Dec(DC);
    dec_AC = ZeroRunDec(AC, block_length);
    dec = merge_coeff(dec_DC,dec_AC);
    decoded_blocks= reshape(dec, [64*num_blocks(1), 3*num_blocks(2)]);    
    inv_zigzag = blockproc(decoded_blocks, [64, 3], @(block_struct) DeZigZag8x8(block_struct.data));
    inv_quant = blockproc(inv_zigzag, [8, 8], @(block_struct) DeQuant8x8(block_struct.data, qScale));
    ycbcr = blockproc(inv_quant, [8, 8], @(block_struct) IDCT8x8(block_struct.data));
    rgb = ictYCbCr2RGB(ycbcr);
    dst = rgb;
end

function [DC,AC] = IntraEncode_DPCM(image, qScale, block_length)
%  Function Name : IntraEncode.m
%  Input         : image (Original RGB Image)
%                  qScale(quantization scale)
%  Output        : dst   (sequences after zero-run encoding, 1xN)
    ycbcr = ictRGB2YCbCr(image);
    I_dct = blockproc(ycbcr, [8, 8], @(block_struct) DCT8x8(block_struct.data));   
    quant = blockproc(I_dct, [8, 8], @(block_struct) Quant8x8(block_struct.data, qScale));   
    zigzag = blockproc(quant, [8, 8], @(block_struct) ZigZag8x8(block_struct.data));   
    before_encoding = zigzag(:)';
    [DC,AC] = split_coeff(before_encoding);
    AC = ZeroRunEnc(AC, block_length);
    DC = DPCM(DC);
end

function dst = IntraEncode_SC(image, qScale, block_length, channel)
%  Function Name : IntraEncode.m
%  Input         : image (Original RGB Image)
%                  qScale(quantization scale)
%  Output        : dst   (sequences after zero-run encoding, 1xN)
    I_dct = blockproc(image, [8, 8], @(block_struct) dct2(block_struct.data));   
    quant = blockproc(I_dct, [8, 8], @(block_struct) Quant8x8_SC(block_struct.data, qScale, channel));   
    zigzag = blockproc(quant, [8, 8], @(block_struct) ZigZag8x8_SC(block_struct.data));   
    before_encoding = zigzag(:)';
    dst = ZeroRunEnc(before_encoding, block_length);
end

function dst = IntraDecode_SC(image, img_size , qScale, block_length, channel)
%  Function Name : IntraDecode.m
%  Input         : image (zero-run encoded image, 1xN)
%                  img_size (original image size)
%                  qScale(quantization scale)
%  Output        : dst   (decoded image)
    num_blocks = img_size ./ 8;
    decoded = ZeroRunDec(image, block_length);
    decoded_blocks= reshape(decoded, [64*num_blocks(1), num_blocks(2)]);    
    inv_zigzag = blockproc(decoded_blocks, [64, 1], @(block_struct) DeZigZag8x8_SC(block_struct.data));
    inv_quant = blockproc(inv_zigzag, [8, 8], @(block_struct) DeQuant8x8_SC(block_struct.data, qScale, channel));
    ycbcr = blockproc(inv_quant, [8, 8], @(block_struct) idct2(block_struct.data));
    dst = ycbcr;
end

function coeff = DCT8x8(block)
%  Input         : block    (Original Image block, 8x8x3)
%
%  Output        : coeff    (DCT coefficients after transformation, 8x8x3)
coeff = zeros(size(block));  % Initialize coefficient matrix

    % Perform DCT on each channel individually
    for channel = 1:3
        coeff(:, :, channel) = dct2(block(:, :, channel));
    end
end

function quant = Quant8x8(dct_block, qScale)
%  Input         : dct_block (Original Coefficients, 8x8x3)
%                  qScale (Quantization Parameter, scalar)
%
%  Output        : quant (Quantized Coefficients, 8x8x3)
matrix=zeros(8,8,3);

L = [16 11 10 16 24 40 51 61;...
    12 12 14 19 26 58 60 55;...
    14 13 16 24 40 57 69 56;...
    14 17 22 29 51 87 80 62;...
    18 55 37 56 68 109 103 77;...
    24 35 55 64 81 104 113 92;...
    49 64 78 87 103 121 120 101;...
    72 92 95 98 112 100 103 99];

C = [17 18 24 47 99 99 99 99;...
    18 21 26 66 99 99 99 99;...
    24 13 56 99 99 99 99 99;...
    47 66 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99];

Base_Q = [ 16  11  10  16  24  40  51  61;
             12  12  14  19  26  58  60  55;
             14  13  16  24  40  57  69  56;
             14  17  22  29  51  87  80  62;
             18  22  37  56  68 109 103  77;
             24  35  55  64  81 104 113  92;
             49  64  78  87 103 121 120 101;
             72  92  95  98 112 100 103  99 ];

Adaptive=[4 6 7 8 10 13 18 31;
        6 9 10 12 15 20 28 48;
        7 10 12 14 18 23 32 55;
        8 12 14 17 21 27 38 65;
        10 15 18 21 26 33 47 80;
        13 20 23 27 33 43 61 103;
        18 28 32 38 47 61 86 146;
        31 48 55 65 80 103 146 250;];


QP=23;
QP_C=24;
% Scaling factor and quantization matrix calculation
Q = floor((Base_Q * QP + 12) / 24);
Q_C=floor((Base_Q * QP_C + 12) / 24);
    
matrix(:,:,1)=Adaptive;
matrix(:,:,2)=C;
matrix(:,:,3)=C;

quant = zeros(8, 8, 3);  % Initialize quantized coefficients matrix

    % Quantize each channel individually
    for channel = 1:3
        for i=1:8
            for j=1:8
                quant(i, j, channel) = round(dct_block(i, j, channel) / (qScale * matrix(i, j, channel)));
            end
        end
    end
end

function quant = Quant8x8_SC(dct_block, qScale, channel)
%  Input         : dct_block (Original Coefficients, 8x8x3)
%                  qScale (Quantization Parameter, scalar)
%
%  Output        : quant (Quantized Coefficients, 8x8x3)
matrix=zeros(8,8,3);

L = [16 11 10 16 24 40 51 61;...
    12 12 14 19 26 58 60 55;...
    14 13 16 24 40 57 69 56;...
    14 17 22 29 51 87 80 62;...
    18 55 37 56 68 109 103 77;...
    24 35 55 64 81 104 113 92;...
    49 64 78 87 103 121 120 101;...
    72 92 95 98 112 100 103 99];

C = [17 18 24 47 99 99 99 99;...
    18 21 26 66 99 99 99 99;...
    24 13 56 99 99 99 99 99;...
    47 66 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99];

Base_Q = [ 16  11  10  16  24  40  51  61;
             12  12  14  19  26  58  60  55;
             14  13  16  24  40  57  69  56;
             14  17  22  29  51  87  80  62;
             18  22  37  56  68 109 103  77;
             24  35  55  64  81 104 113  92;
             49  64  78  87 103 121 120 101;
             72  92  95  98 112 100 103  99 ];

Adaptive=[4 6 7 8 10 13 18 31;
        6 9 10 12 15 20 28 48;
        7 10 12 14 18 23 32 55;
        8 12 14 17 21 27 38 65;
        10 15 18 21 26 33 47 80;
        13 20 23 27 33 43 61 103;
        18 28 32 38 47 61 86 146;
        31 48 55 65 80 103 146 250;];


QP=23;
QP_C=24;
% Scaling factor and quantization matrix calculation
Q = floor((Base_Q * QP + 12) / 24);
Q_C=floor((Base_Q * QP_C + 12) / 24);
    
matrix(:,:,1)=Adaptive;
matrix(:,:,2)=C;
matrix(:,:,3)=C;

quant = zeros(8, 8, 1);  % Initialize quantized coefficients matrix

    % Quantize each channel individually
    for i=1:8
        for j=1:8
            quant(i, j) = round(dct_block(i, j) / (qScale * matrix(i, j, channel)));
        end
    end
end

function zz = ZigZag8x8(quant)
%  Input         : quant (Quantized Coefficients, 8x8x3)
%
%  Output        : zz (zig-zag scaned Coefficients, 64x3)
% Zig-zag scanning matrix
    zigzagIndices = [
        1 2 6 7 15 16 28 29;
        3 5 8 14 17 27 30 43;
        4 9 13 18 26 31 42 44;
        10 12 19 25 32 41 45 54;
        11 20 24 33 40 46 53 55;
        21 23 34 39 47 52 56 61;
        22 35 38 48 51 57 60 62;
        36 37 49 50 58 59 63 64
    ];
    zigzagIndices=zigzagIndices(:);

    zz = zeros(64, 3);  % Initialize zig-zag scanned coefficients matrix

     for channel = 1:3
        % Perform zig-zag scanning on the quantized coefficients for each channel
        quantChannel = quant(:, :, channel); 
        quantChannel = quantChannel(:);
        for i = 1:64
            zz(zigzagIndices(i), channel) = quantChannel(i);
        end
    end
end

function zz = ZigZag8x8_SC(quant)
%  Input         : quant (Quantized Coefficients, 8x8x3)
%
%  Output        : zz (zig-zag scaned Coefficients, 64x3)
% Zig-zag scanning matrix
    zigzagIndices = [
        1 2 6 7 15 16 28 29;
        3 5 8 14 17 27 30 43;
        4 9 13 18 26 31 42 44;
        10 12 19 25 32 41 45 54;
        11 20 24 33 40 46 53 55;
        21 23 34 39 47 52 56 61;
        22 35 38 48 51 57 60 62;
        36 37 49 50 58 59 63 64
    ];
    zigzagIndices=zigzagIndices(:);

    zz = zeros(64, 1);  % Initialize zig-zag scanned coefficients matrix

    % Perform zig-zag scanning on the quantized coefficients for each channel
    quantChannel = quant(:, :); 
    quantChannel = quantChannel(:);
    for i = 1:64
        zz(zigzagIndices(i)) = quantChannel(i);
    end
end

function zze = ZeroRunEnc(zz, block_length)


%  Input         : zz (Zig-zag scanned sequence, 1xN)
%                  EOB (End Of Block symbol, scalar)
%
%  Output        : zze (zero-run-level encoded sequence, 1xM)
    zze = [];  % Initialize the zero-run-level encoded sequence
    zeroCount = 0;  % Count of consecutive zero values
    count = 0;
    EOB=6000;
    
    for i = 1:numel(zz)
        if zz(i) == 0
            zeroCount = zeroCount + 1;
            if mod(i,block_length)==0
                zeroCount=0;
                count=0;
                zze = [zze EOB];
                continue
            end
        else
            % Append the 0,zero count and the non-zero value to the encoded sequence if the last value was a 0
            if i>1 && zz(i-1) == 0 && zeroCount~=0 
            zze = [zze 0 zeroCount-1 zz(i)];
            % Append just the non-zero value if the last value wasn't a 0
            else
                zze = [zze zz(i)];
            end
            if mod(i,block_length)==0
                zeroCount=0;
                count=0;
                continue
            end
            zeroCount = 0;  % Reset the zero count
        end
    end

end

function [DC,AC] = split_coeff(seq)
    DC=seq(1:64:end);
    positionsToRemove = 1:64:length(seq);
    mask = true(size(seq));
    mask(positionsToRemove) = false;
    AC=seq(mask);
end

function merged=merge_coeff(dec_DC,dec_AC)
    merged=zeros(1,numel(dec_DC)+numel(dec_AC));
    for i = 1:numel(dec_DC)
        merged((i-1)*64 + 1) = dec_DC(i); 
        merged((i-1)*64 + 2:i*64) = dec_AC((i-1)*63 + 1:i*63); 
    end
end

function DC = DPCM(DC)
    substract=[0,DC(1:end-1)];
    DC=DC-substract;
end

function DC = DPCM_Dec(DC)
    for i=2:length(DC)
        DC(i)=DC(i-1)+DC(i);
    end
end

function dst = ZeroRunDec(src, block_length)

%  Function Name : ZeroRunDec1.m zero run level decoder
%  Input     	: src (zero run encoded sequence 1xM with EoB signs)
%              	EoB (end of block sign)
%
%  Output    	: dst (reconstructed zig-zag scanned sequence 1xN)
count = 1;
i=1;
dst=[];
EoB=6000;
while i<=numel(src) 
    if (i == 1 || (i>1 && src(i-1)~=0)|| (i>2 && src(i-2)==0)) && src(i)~=EoB
    	dst(count) = src(i);
    	count = count+1;
    elseif (i>1 && src(i-1) == 0 && src(i) ==0)
        count=count;
	elseif ((i>2 && src(i-1) == 0 && src(i-2)~=0) || (i>1 && src(i-1) == 0)) && src(i)~=EoB
        dst(count:count+src(i)-1)=0;
        count=count+src(i);
    elseif src(i)==EoB
    	len = length(dst);
    	left = block_length-mod(len,block_length);
        dst(count:count+left-1)=0;
        count=count+left;
    end
    i=i+1;
end
end

function dct_block = DeQuant8x8(quant_block, qScale)
%  Function Name : DeQuant8x8.m
%  Input         : quant_block  (Quantized Block, 8x8x3)
%                  qScale       (Quantization Parameter, scalar)
%
%  Output        : dct_block    (Dequantized DCT coefficients, 8x8x3)
matrix=zeros(8,8,3);

L = [16 11 10 16 24 40 51 61;...
    12 12 14 19 26 58 60 55;...
    14 13 16 24 40 57 69 56;...
    14 17 22 29 51 87 80 62;...
    18 55 37 56 68 109 103 77;...
    24 35 55 64 81 104 113 92;...
    49 64 78 87 103 121 120 101;...
    72 92 95 98 112 100 103 99];

C = [17 18 24 47 99 99 99 99;...
    18 21 26 66 99 99 99 99;...
    24 13 56 99 99 99 99 99;...
    47 66 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99];

Base_Q = [ 16  11  10  16  24  40  51  61;
             12  12  14  19  26  58  60  55;
             14  13  16  24  40  57  69  56;
             14  17  22  29  51  87  80  62;
             18  22  37  56  68 109 103  77;
             24  35  55  64  81 104 113  92;
             49  64  78  87 103 121 120 101;
             72  92  95  98 112 100 103  99 ];

Adaptive=[4 6 7 8 10 13 18 31;
        6 9 10 12 15 20 28 48;
        7 10 12 14 18 23 32 55;
        8 12 14 17 21 27 38 65;
        10 15 18 21 26 33 47 80;
        13 20 23 27 33 43 61 103;
        18 28 32 38 47 61 86 146;
        31 48 55 65 80 103 146 250;];


QP=23;
QP_C=24;
% Scaling factor and quantization matrix calculation
Q = floor((Base_Q * QP + 12) / 24);
Q_C=floor((Base_Q * QP_C + 12) / 24);
    
matrix(:,:,1)=Adaptive;
matrix(:,:,2)=C;
matrix(:,:,3)=C;

dct_block = zeros(8, 8, 3);  % Initialize quantized coefficients matrix

    % Dequantize each channel individually
    for channel = 1:3
        for i=1:8
            for j=1:8
                dct_block(i, j, channel) = quant_block(i,j,channel)*qScale*matrix(i,j,channel);
            end
        end
    end
end

function dct_block = DeQuant8x8_SC(quant_block, qScale, channel)
%  Function Name : DeQuant8x8.m
%  Input         : quant_block  (Quantized Block, 8x8x3)
%                  qScale       (Quantization Parameter, scalar)
%
%  Output        : dct_block    (Dequantized DCT coefficients, 8x8x3)
matrix=zeros(8,8,3);

L = [16 11 10 16 24 40 51 61;...
    12 12 14 19 26 58 60 55;...
    14 13 16 24 40 57 69 56;...
    14 17 22 29 51 87 80 62;...
    18 55 37 56 68 109 103 77;...
    24 35 55 64 81 104 113 92;...
    49 64 78 87 103 121 120 101;...
    72 92 95 98 112 100 103 99];

C = [17 18 24 47 99 99 99 99;...
    18 21 26 66 99 99 99 99;...
    24 13 56 99 99 99 99 99;...
    47 66 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99;...
    99 99 99 99 99 99 99 99];

Base_Q = [ 16  11  10  16  24  40  51  61;
             12  12  14  19  26  58  60  55;
             14  13  16  24  40  57  69  56;
             14  17  22  29  51  87  80  62;
             18  22  37  56  68 109 103  77;
             24  35  55  64  81 104 113  92;
             49  64  78  87 103 121 120 101;
             72  92  95  98 112 100 103  99 ];

Adaptive=[4 6 7 8 10 13 18 31;
        6 9 10 12 15 20 28 48;
        7 10 12 14 18 23 32 55;
        8 12 14 17 21 27 38 65;
        10 15 18 21 26 33 47 80;
        13 20 23 27 33 43 61 103;
        18 28 32 38 47 61 86 146;
        31 48 55 65 80 103 146 250;];


QP=23;
QP_C=24;
% Scaling factor and quantization matrix calculation
Q = floor((Base_Q * QP + 12) / 24);
Q_C=floor((Base_Q * QP_C + 12) / 24);
    
matrix(:,:,1)=Adaptive;
matrix(:,:,2)=C;
matrix(:,:,3)=C;

dct_block = zeros(8, 8, 1);  % Initialize quantized coefficients matrix

    % Dequantize each channel individually
    for i=1:8
        for j=1:8
            dct_block(i, j) = quant_block(i,j)*qScale*matrix(i,j,channel);
        end
    end
end

function block = IDCT8x8(coeff)
%  Function Name : IDCT8x8.m
%  Input         : coeff (DCT Coefficients) 8*8*3
%  Output        : block (original image block) 8*8*3
block = zeros(8, 8, 3);  % Initialize reconstructed image block

    % Reconstruct each channel individually
    for channel = 1:3
        % Transpose the result and perform inverse DCT again
        block(:, :, channel) = idct2(coeff(:, :, channel));
    end
end

function coeffs = DeZigZag8x8(zz)
%  Function Name : DeZigZag8x8.m
%  Input         : zz    (Coefficients in zig-zag order)
%
%  Output        : coeffs(DCT coefficients in original order)
zigzagIndices = [
1 2 6 7 15 16 28 29;
3 5 8 14 17 27 30 43;
4 9 13 18 26 31 42 44;
10 12 19 25 32 41 45 54;
11 20 24 33 40 46 53 55;
21 23 34 39 47 52 56 61;
22 35 38 48 51 57 60 62;
36 37 49 50 58 59 63 64];
coeffs=zeros(8,8,3);
for channel = 1:3
    % Perform zig-zag scanning on the quantized coefficients for each channel
    zzChannel = zz(:, channel); 
    coeffs(:,:,channel) = zzChannel(zigzagIndices);
end
end

function coeffs = DeZigZag8x8_SC(zz)
%  Function Name : DeZigZag8x8.m
%  Input         : zz    (Coefficients in zig-zag order)
%
%  Output        : coeffs(DCT coefficients in original order)
    zigzagIndices = [
    1 2 6 7 15 16 28 29;
    3 5 8 14 17 27 30 43;
    4 9 13 18 26 31 42 44;
    10 12 19 25 32 41 45 54;
    11 20 24 33 40 46 53 55;
    21 23 34 39 47 52 56 61;
    22 35 38 48 51 57 60 62;
    36 37 49 50 58 59 63 64];
    coeffs=zeros(8,8,1);
    % Perform zig-zag scanning on the quantized coefficients
    zzChannel = zz(:); 
    coeffs(:,:) = zzChannel(zigzagIndices);
end

function yuv = ictRGB2YCbCr(rgb)
% Input         : rgb (Original RGB Image)
% Output        : yuv (YCbCr image after transformation)
% YOUR CODE HERE
y=0.299*rgb(:,:,1)+0.587*rgb(:,:,2)+0.114*rgb(:,:,3);
u=-0.169*rgb(:,:,1)-0.331*rgb(:,:,2)+0.5*rgb(:,:,3);
v=0.5*rgb(:,:,1)-0.419*rgb(:,:,2)-0.081*rgb(:,:,3);
yuv=zeros(size(rgb));
yuv(:,:,1)=y;
yuv(:,:,2)=u;
yuv(:,:,3)=v;
end

function rgb = ictYCbCr2RGB(yuv)
% Input         : yuv (Original YCbCr image)
% Output        : rgb (RGB Image after transformation)
% YOUR CODE HERE
rgb(:,:,1)=yuv(:,:,1)+1.402*yuv(:,:,3);
rgb(:,:,2)=yuv(:,:,1)-0.344*yuv(:,:,2)-0.714*yuv(:,:,3);
rgb(:,:,3)=yuv(:,:,1)+1.772*yuv(:,:,2);
end

function pmf = stats_marg(image, range)
pmf = hist(image(:), range);
pmf = pmf + 0.00000000000000000000001;
pmf = pmf/sum(pmf);
end

function PSNR = calcPSNR(Image, recImage)
% Input         : Image    (Original Image)
%                 recImage (Reconstructed Image)
%
% Output        : PSNR     (Peak Signal to Noise Ratio)
% YOUR CODE HERE
% call calcMSE to calculate MSE
MSE=calcMSE(Image,recImage);
PSNR=10*log10((2^8-1)^2/MSE);
end

function MSE = calcMSE(Image, recImage)
% Input         : Image    (Original Image)
%                 recImage (Reconstructed Image)
% Output        : MSE      (Mean Squared Error)
% YOUR CODE HERE
Image=double(Image);
recImage=double(recImage);
MSE=sum((Image(:)-recImage(:)).^2)/(length(Image(:,1,1))*length(Image(1,:,1))*length(Image(1,1,:)));
end

function motion_vectors_indices = SSD(ref_image, image)
%  Input         : ref_image(Reference Image, size: height x width)
%                  image (Current Image, size: height x width)
%
%  Output        : motion_vectors_indices (Motion Vector Indices, size: (height/8) x (width/8) x 1 )
best_ssd=10000000000000;
motion_vectors_indices=zeros(size(image,1)/8,size(image,2)/8);
for i=1:8:size(image,1)
    for j=1:8:size(image,2)
        if i+4>size(image,1)
            continue
        end
        if j+4>size(image,2)
            continue
        end
        cur_block=image(i:i+7,j:j+7);
        for k=-4:4
            for l=-4:4
                if i+k<1  
                    continue
                end
                if j+l<1
                    continue
                end
                if i+k+7>size(image,1)
                   continue
                end
                if j+l+7>size(image,2)
                    continue
                end
                ref_block=ref_image(i+k:i+k+7,j+l:j+l+7);
                ssd=sum(sum((cur_block-ref_block).^2));
                if ssd<best_ssd
                    best_ssd=ssd;
                    index=(k+4)*9+(l+4)+1;
                end
            end
        end
        motion_vectors_indices((i+7)/8,(j+7)/8)=index;
        best_ssd=10000000000000;
    end
end
end

function rec_image = SSD_rec(ref_image, motion_vectors)
%  Input         : ref_image(Reference Image, YCbCr image)
%                  motion_vectors
%
%  Output        : rec_image (Reconstructed current image, YCbCr image)
rec_image=zeros(size(ref_image));
for i=1:8:size(ref_image,1)
    for j=1:8:size(ref_image,2)
        for k=1:3
            cur_vector=motion_vectors((i+7)/8,(j+7)/8);
            col=mod(cur_vector-1,9)-4;
            row=floor((cur_vector-1)/9)-4;
            r = i+row:i+row+7;
            c = j+col:j+col+7;
            rec_image(i:i+7,j:j+7,k)=ref_image(r,c,k);
        end
    end
end
end

function motionVector2d = LogMotionEstimation_new(referenceFrame, currentFrame, range)
    % currentFrame: The current frame
    % referenceFrame: The reference frame
    % blockSize: The size of the block (assumed to be square, e.g., 8x8)
    % searchRange: The initial search range around the current block center (assumed to be square)
    
    blockSize=8;
    searchRange=4;
    [frameHeight, frameWidth] = size(referenceFrame);
    numBlocksY = floor(frameHeight / blockSize);
    numBlocksX = floor(frameWidth / blockSize);
    motionVectors = zeros(numBlocksY, numBlocksX,2);

    for i = 1 : numBlocksY
        for j = 1 : numBlocksX
            % Extract the current block from the current frame
            currentBlock = currentFrame((i - 1) * blockSize + 1 : i * blockSize, (j - 1) * blockSize + 1 : j * blockSize);

            % Perform the Three Step Search block matching for the current block
            % Initialize the minimum error and motion vector
            minError = inf;
            motionVector = [0, 0];
            halfSearchRange = floor(searchRange / 2);

            % Loop through all possible motion vectors within the search range
            while halfSearchRange >= 1
                for dy = -halfSearchRange : halfSearchRange: halfSearchRange
                    for dx = -halfSearchRange : halfSearchRange: halfSearchRange
                        % Calculate the candidate motion vector
                        if abs(dx)==abs(dy) && dx~=0
                            continue
                        end
                        candidateMV = [dy, dx];

                        % Calculate the candidate block position in the reference frame
                        candidateBlockPos = motionVector + candidateMV;
        
                        % Check if the candidate block position is within the reference frame
                        if candidateBlockPos(1)+(i-1)*blockSize >= 0 && candidateBlockPos(1)+i*blockSize <= frameHeight ...
                                && candidateBlockPos(2)+(j-1)*blockSize >=0 && candidateBlockPos(2) + j*blockSize <= frameWidth
                            
                            % Extract the candidate block from the reference frame
                            candidateBlock = referenceFrame(candidateBlockPos(1)+(i-1)*blockSize+1 : candidateBlockPos(1)+i*blockSize, ...
                                candidateBlockPos(2)+(j-1)*blockSize+1: candidateBlockPos(2) + j*blockSize);
        
                            % Calculate the mean squared error (MSE) between the current block and the candidate block
                            ssd = sum(sum((currentBlock - candidateBlock) .^ 2));
        
                            % Update the motion vector if the error is smaller
                            if ssd < minError
                                minError = ssd;
                                motionVectorItteration = candidateMV;
                            end
                        end

                    end
                end
                % Reduce the search range for the next iteration
                if (motionVectorItteration(1)==0 && motionVectorItteration(2)==0)
                    halfSearchRange=halfSearchRange/2;
                end
                motionVector=motionVector+motionVectorItteration;
                motionVectorItteration=[0,0];
                if abs(motionVector(1))==range-1 || abs(motionVector(2))==range-1 || abs(motionVector(1))==range || abs(motionVector(2))==range
                    halfSearchRange=halfSearchRange/2;
                end
            end
            % Store the motion vector for the current block
            motionVectors(i, j, :) = motionVector;
        end
    end
    motionVector2d = (motionVectors(:,:,1)+range)*(range*2+1)+(motionVectors(:,:,2)+range)+1;
end

function rec_image = Log_rec_new(ref_image, motion_vectors, range)
%  Input         : ref_image(Reference Image, YCbCr image)
%                  motion_vectors
%
%  Output        : rec_image (Reconstructed current image, YCbCr image)
rec_image=zeros(size(ref_image));
for i=1:8:size(ref_image,1)
    for j=1:8:size(ref_image,2)
        for k=1:3
            cur_vector=motion_vectors((i+7)/8,(j+7)/8);
            col=mod(cur_vector-1,range*2+1)-range;
            row=floor((cur_vector-1)/(range*2+1))-range;
            r = i+row:i+row+7;
            c = j+col:j+col+7;
            % i
            % j
            % cur_vector
            rec_image(i:i+7,j:j+7,k)=ref_image(r,c,k);
        end
    end
end
end

function motionVectors = ThreeStepMotionEstimation(referenceFrame, currentFrame)
    % currentFrame: The current frame
    % referenceFrame: The reference frame
    % blockSize: The size of the block (assumed to be square, e.g., 8x8)
    % searchRange: The initial search range around the current block center (assumed to be square)
    
    blockSize=8;
    searchRange=8;
    [frameHeight, frameWidth] = size(referenceFrame);
    numBlocksY = floor(frameHeight / blockSize);
    numBlocksX = floor(frameWidth / blockSize);
    motionVectors = zeros(numBlocksY, numBlocksX);

    for i = 1 : numBlocksY
        for j = 1 : numBlocksX
            % Extract the current block from the current frame
            currentBlock = currentFrame((i - 1) * blockSize + 1 : i * blockSize, (j - 1) * blockSize + 1 : j * blockSize);

            % Perform the Three Step Search block matching for the current block
            % Initialize the minimum error and motion vector
            minError = inf;
            motionVector = [0, 0];
            halfSearchRange = floor(searchRange / 2);

            % Loop through all possible motion vectors within the search range
            while halfSearchRange >= 1
                for dy = -halfSearchRange : halfSearchRange: halfSearchRange
                    for dx = -halfSearchRange : halfSearchRange: halfSearchRange
                        % Calculate the candidate motion vector
                        candidateMV = [dy, dx];
        
                        % Calculate the candidate block position in the reference frame
                        candidateBlockPos = motionVector + candidateMV;
        
                        % Check if the candidate block position is within the reference frame
                        if candidateBlockPos(1)+(i-1)*blockSize >= 0 && candidateBlockPos(1)+i*blockSize <= frameHeight ...
                                && candidateBlockPos(2)+(j-1)*blockSize >=0 && candidateBlockPos(2) + j*blockSize <= frameWidth
                            
                            % Extract the candidate block from the reference frame
                            candidateBlock = referenceFrame(candidateBlockPos(1)+(i-1)*blockSize+1 : candidateBlockPos(1)+i*blockSize, ...
                                candidateBlockPos(2)+(j-1)*blockSize+1: candidateBlockPos(2) + j*blockSize);
        
                            % Calculate the mean squared error (MSE) between the current block and the candidate block
                            ssd = sum(sum((currentBlock - candidateBlock) .^ 2));
        
                            % Update the motion vector if the error is smaller
                            if ssd < minError
                                minError = ssd;
                                motionVectorItteration = candidateMV;
                            end
                        end

                    end
                end
                % Reduce the search range for the next iteration
                motionVector=motionVector+motionVectorItteration;
                motionVectorItteration=[0,0];
                halfSearchRange = halfSearchRange / 2;
            end
            % Store the motion vector for the current block
            index=motionVector(2)+8+(motionVector(1)+7)*15;
            motionVectors(i, j) = index;
        end
    end
end

function rec_image = ThreeStep_Rec(ref_image, motion_vectors)
%  Input         : ref_image(Reference Image, YCbCr image)
%                  motion_vectors
%
%  Output        : rec_image (Reconstructed current image, YCbCr image)
rec_image=zeros(size(ref_image));
for i=1:8:size(ref_image,1)
    for j=1:8:size(ref_image,2)
        for k=1:3
            cur_vector=motion_vectors((i+7)/8,(j+7)/8);
            col=mod(cur_vector-1,15)-7;
            row=floor((cur_vector-1)/15)-7;
            r = i+row:i+row+7;
            c = j+col:j+col+7;
            rec_image(i:i+7,j:j+7,k)=ref_image(r,c,k);
        end
    end
end
end

function [ BinaryTree, HuffCode, BinCode, Codelengths] = buildHuffman( p )

global y

p=p(:)/sum(p)+eps;              % normalize histogram
p1=p;                           % working copy

c=cell(length(p1),1);			% generate cell structure 

for i=1:length(p1)				% initialize structure
   c{i}=i;						
end

while size(c)-2					% build Huffman tree
	[p1,i]=sort(p1);			% Sort probabilities
	c=c(i);						% Reorder tree.
	c{2}={c{1},c{2}};           % merge branch 1 to 2
    c(1)=[];	                % omit 1
	p1(2)=p1(1)+p1(2);          % merge Probabilities 1 and 2 
    p1(1)=[];	                % remove 1
end

%cell(length(p),1);              % generate cell structure
getcodes(c,[]);                  % recurse to find codes
code=char(y);

[numCodes maxlength] = size(code); % get maximum codeword length

% generate byte coded huffman table
% code

length_b=0;
HuffCode=zeros(1,numCodes);
for symbol=1:numCodes
    for bit=1:maxlength
        length_b=bit;
        if(code(symbol,bit)==char(49)) HuffCode(symbol) = HuffCode(symbol)+2^(bit-1)*(double(code(symbol,bit))-48);
        elseif(code(symbol,bit)==char(48))
        else 
            length_b=bit-1;
            break;
        end
    end
    Codelengths(symbol)=length_b;
end

BinaryTree = c;
BinCode = code;

clear global y;

return
end

function getcodes(a,dum)
global y                            % in every level: use the same y
if isa(a,'cell')                    % if there are more branches...go on
         getcodes(a{1},[dum 0]);    % 
         getcodes(a{2},[dum 1]);
else   
   y{a}=char(48+dum);   
end
end

function [output] = dec_huffman_new (bytestream, BinaryTree, nr_symbols)

output = zeros(1,nr_symbols);
ctemp = BinaryTree;

dec = zeros(size(bytestream,1),8);
for i = 8:-1:1
    dec(:,i) = rem(bytestream,2);
    bytestream = floor(bytestream/2);
end

dec = dec(:,end:-1:1)';
a = dec(:);

i = 1;
p = 1;
while(i <= nr_symbols)&&p<=max(size(a))
    while(isa(ctemp,'cell'))
        next = a(p)+1;
        p = p+1;
        ctemp = ctemp{next};
    end
    output(i) = ctemp;
    ctemp = BinaryTree;
    i=i+1;
end
end

function [bytestream] = enc_huffman_new( data, BinCode, Codelengths)

a = BinCode(data(:),:)';
b = a(:);
mat = zeros(ceil(length(b)/8)*8,1);
p  = 1;
for i = 1:length(b)
    if b(i)~=' '
        mat(p,1) = b(i)-48;
        p = p+1;
    end
end
p = p-1;
mat = mat(1:ceil(p/8)*8);
d = reshape(mat,8,ceil(p/8))';
multi = [1 2 4 8 16 32 64 128];
bytestream = sum(d.*repmat(multi,size(d,1),1),2);

end
