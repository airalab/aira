import { bindActionCreators } from 'redux'
import { reduxForm } from 'redux-form'
import _ from 'lodash'
import { submitIdentify } from '../../../modules/app/actions';
import Form from '../../../shared/components/app/form';

const validate = (values) => {
  const errors = {};
  if (!values.code) {
    errors.code = 'required'
  }
  if (!values.value) {
    errors.value = 'required'
  } else if (_.toNumber(values.value) > 20) {
    errors.value = 'max 20'
  }
  return errors
};
function mapStateToProps() {
  return {
    fields: ['code', 'value'],
    labels: ['Verification code', 'ETH amount for transfer to Aira ether funds smart contract'],
    placeholders: ['xxxxxx', '0']
  }
}
function mapDispatchToProps(dispatch) {
  return {
    onSubmit: bindActionCreators(submitIdentify, dispatch)
  }
}
export default reduxForm({
  form: 'FormIdentify',
  validate
}, mapStateToProps, mapDispatchToProps)(Form)
