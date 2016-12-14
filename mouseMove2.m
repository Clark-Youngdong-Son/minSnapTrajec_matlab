function mouseMove2(object, eventdata)
    global count m;
    if(count~=m)
        C = get(gca, 'CurrentPoint');
        title(gca,['Altitude = (', num2str(C(1,2),2), ')']);
    end
end