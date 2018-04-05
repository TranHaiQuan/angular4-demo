require 'test_helper'

class UsersControllerTest < ActionDispatch::IntegrationTest
  test "should get list_users" do
    get users_list_users_url
    assert_response :success
  end

end
