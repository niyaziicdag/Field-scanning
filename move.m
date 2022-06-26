function [xy_state] =move(xy_state,k,birey,Mx)
directions=[0 0; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1 ; 1 0; 1 1];
    p_kxy=xy_state+directions(birey(k)+1,:); 
    if p_kxy(1)<1 || p_kxy(2)<1 || p_kxy(1)>Mx  || p_kxy(2)>Mx
    else 
          xy_state=p_kxy;
    end
    M(xy_state(1),xy_state(2))=k+1;
end