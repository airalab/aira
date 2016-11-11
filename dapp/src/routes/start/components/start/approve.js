import React from 'react'
import FormApprove from '../../containers/formApprove'

const Approve = props => (
  <div>
    <h1>Approve @AiraEthBot for /create via Telegram</h1>
    <p>You have ETH on Aira ether funds contract: {props.balance} ETH</p>
    <FormApprove approved={props.approved} />
  </div>
)

export default Approve
