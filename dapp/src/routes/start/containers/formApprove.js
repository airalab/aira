import { bindActionCreators } from 'redux'
import { reduxForm } from 'redux-form'
import { submitApprove, getApprovedByAddress, getBalanceByToken } from '../../../modules/app/actions';
import Form from '../../../shared/components/app/form';
import { ADDRESS, ADDRESS_BOT, ADDRESS_ST, ADDRESS_TE } from '../../../config/config'

const validate = (values) => {
  const errors = {};
  if (!values.value) {
    errors.value = 'required'
  }
  return errors
};
function mapStateToProps(state, props) {
  return {
    fields: ['address_token', 'address_target', 'value'],
    labels: ['Token address contract. You balance: ' + props.balance, 'Target address. Approved: ' + props.approved, 'How much tokens you want approve'],
    placeholders: ['Token address contract', 'Target address', '0.1'],
    autocomplete: {
      address_token: [
        {
          title: 'Aira Ether Funds',
          value: ADDRESS
        },
        {
          title: 'Other',
          value: ''
        }
      ],
      address_target: [
        {
          title: '@AiraEthBot',
          value: ADDRESS_BOT
        },
        {
          title: 'Standart token',
          value: ADDRESS_ST
        },
        {
          title: 'Token with emission',
          value: ADDRESS_TE
        },
        {
          title: 'Other',
          value: ''
        }
      ]
    }
  }
}
function mapDispatchToProps(dispatch) {
  const actions = bindActionCreators({
    submitApprove,
    getApprovedByAddress,
    getBalanceByToken
  }, dispatch)
  return {
    onSubmit: actions.submitApprove,
    onChangeInput: {
      address_token: (value) => {
        actions.getBalanceByToken(value)
      },
      address_target: (value, form) => {
        actions.getApprovedByAddress(form.address_token, value)
      }
    }
  }
}
export default reduxForm({
  form: 'FormApprove',
  validate
}, mapStateToProps, mapDispatchToProps)(Form)
