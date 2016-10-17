import React from 'react'
import FormApprove from '../../containers/formApprove'

const Approve = props => (
  <div>
    <h1>Approve @AiraEthBot for send ETH via Telegram</h1>
    <p>You have ETH on Aira ether funds contract: {props.balance} ETH</p>
    <p>@AiraEthBot approved: {props.approved} ETH</p>
    <FormApprove />
  </div>
)

export default Approve
