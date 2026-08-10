function [nfData,newWeight,newInvConv] = rls_engine(fData,refData,oldWeight,oldInvConv,lambda)
    scaledSig = oldWeight.*fData;
    errsig = refData - scaledSig;
    % RLS algorithm
    XP = conj(fData).*oldInvConv;
    invDen = 1./(lambda + XP.*fData);
    K = invDen.*(oldInvConv.*fData);
    newInvConv = (1/lambda)*(oldInvConv - K.*XP);
    newWeight = oldWeight + errsig .* conj(K);
    nfData = fData.*oldWeight;
end

