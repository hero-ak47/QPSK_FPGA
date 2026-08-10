function [nfData,newWeight,newInvConv] = rls_engine2(fData,refData,oldWeight,oldInvConv,lambda)
    % Apply old weight
    scaledSig = oldWeight .* fData;

    % Error signal
    errsig = refData - scaledSig;

    % RLS internal variables
    XP = conj(fData) .* oldInvConv;
    invDen = 1 ./ (lambda + XP .* fData);
    K = invDen .* (oldInvConv .* fData);

    % Update inverse covariance matrix
    newInvConv = (1/lambda)*(oldInvConv - K .* XP);

    % -------- PHASE-ONLY UPDATE --------
    % Step 1: update weight like standard RLS
    w_temp = oldWeight + errsig .* conj(K);

    % Step 2: force |w| = 1 (unit magnitude)
    newWeight = w_temp ./ (abs(w_temp) + eps);

    % Output signal with updated (phase-only) weight
    nfData = fData .* newWeight;
end

