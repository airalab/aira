
operationalFee :: Double
operationalFee = 0.02

withFee :: AEF.SHA3 -> Double -> Double -> Web3 Text -> Web3 Text
withFee client fee value fun = do
    beneficiary <- resolve "AiraEth.bot"
    balance     <- AEF.getBalance client
    if fee + value > balance
    then throwError $ UserFail "Allowed balance is too low for this operation!"
    else AEF.sendFrom client beneficiary fee >> fun
