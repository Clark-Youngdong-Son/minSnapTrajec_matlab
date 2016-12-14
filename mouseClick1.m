function mouseClick1(object, eventdata)
    global count m keyframe;

    if(count < m)
        count = count + 1;
        C = get(gca, 'CurrentPoint');
        keyframe(1,count) = C(1,1);               %x
        keyframe(2,count) = C(1,2);               %y
        plot(keyframe(1,count), keyframe(2,count), 'ro','MarkerFaceColor', 'r');
        text(keyframe(1,count)+0.6, keyframe(2,count)+0.6, num2str(count));
    end
end