import { bindActionCreators } from 'redux'
import { reduxForm } from 'redux-form'
import _ from 'lodash'
import { submitApprove, getApprovedByAddress } from '../../../modules/app/actions';
import Form from '../../../shared/components/app/form';

const validate = (values) => {
  const errors = {};
  if (!values.value) {
    errors.value = 'required'
  } else if (_.toNumber(values.value) > 20) {
    errors.value = 'max 20'
  }
  return errors
};
function mapStateToProps(state, props) {
  return {
    fields: ['value', 'address'],
    selects: {
      address: [
        {
          name: '--- select contract ---',
          value: ''
        },
        {
          name: 'Standart token',
          value: '0xb3afb61beb834242ec01f6bbd6f178cc4860c2bb'
        },
        {
          name: 'Token with emission',
          value: '0x733976c3245953420a69efae41fd3c7553233710'
        }
      ]
    },
    labels: ['How much ETH you want approve', 'Contract approved: ' + props.approved + ' ETH'],
    placeholders: ['0.1']
  }
}
function mapDispatchToProps(dispatch) {
  return {
    onSubmit: bindActionCreators(submitApprove, dispatch),
    onChangeSelect: bindActionCreators(getApprovedByAddress, dispatch)
  }
}
export default reduxForm({
  form: 'FormApprove',
  validate
}, mapStateToProps, mapDispatchToProps)(Form)
