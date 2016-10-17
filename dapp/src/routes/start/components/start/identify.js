/* eslint react/jsx-no-target-blank: 0 */
import React from 'react'
import FormIdentify from '../../containers/formIdentify'

const Identify = () => (
  <div>
    <h1>Identify of Telegram account - Ethereum network address</h1>
    <FormIdentify />
    <blockquote>
      <p>
        After send transaction please wait before <a href="https://web.telegram.org/#/im?p=@AiraEthBot" target="_blank">@AiraEthBot</a> send you message with
        successful verify of Telegram nickname - Ethereum network address.
        You can check verify status themselves with command “/me” on chat with <a href="https://web.telegram.org/#/im?p=@AiraEthBot" target="_blank">@AiraEthBot</a>.
      </p>
      <p>If you have any questions, please write on welcome@aira.life</p>
    </blockquote>
  </div>
)

export default Identify
