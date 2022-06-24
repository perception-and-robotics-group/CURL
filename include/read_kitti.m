function data = read_kitti(fileName)
    fileKitti = fopen(fileName,'r');
    data =  fread(fileKitti,'single');
    fclose(fileKitti);
    data = reshape(data,4,[])';
    data = data(:,1:3);
end

