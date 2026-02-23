function MD0= mydata_mk_struct( worldLines, imgLines, worldPts, imgPts )
MD0.Pts= worldPts;
MD0.pts= imgPts;
for i=1:length(worldLines)
    MD0.(sprintf('Line%d',i))= worldLines{i};
    MD0.(sprintf('line%d',i))= imgLines{i};
end
