function mouseMove1(object, eventdata)
    global count m;
    if(count~=m)
        C = get(gca, 'CurrentPoint');
        title(gca,['(X,Y) = (', num2str(C(1,1),2), ', ',num2str(C(1,2),2), ')']);
    end
end